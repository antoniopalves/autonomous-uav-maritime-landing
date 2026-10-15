#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

RATE_HZ = 20.0

class KalmanFilter2D:
    def __init__(self):
        self.x = np.zeros((4, 1))
        self.P = np.eye(4)
        self.Q = np.eye(4) * 0.2
        self.R = np.eye(2) * 0.3
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        self.is_initialized = False

    def predict(self, dt):
        if not self.is_initialized: return
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        if not self.is_initialized:
            self.x[0, 0] = z[0, 0]
            self.x[1, 0] = z[1, 0]
            self.is_initialized = True
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P


class AutonomousLandingFinal(Node):
    def __init__(self):
        super().__init__('land_debug')

        # --- SELETOR DE MODO ---
        self.boat_mode = 2  # 1 = Barco Parado | 2 = Barco em Movimento
        # -----------------------

        self.target_gps_xy = (-87.0, 10.0)
        self.search_alt = 5.0
        self.coarse_alt = 2.0
        self.landing_threshold_alt = 0.35
        self.precision_threshold = 1.20
        self.blind_landing_alt = 1.20
        self.lead_time = 1.2

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state = State()
        self.current_pose = None
        self.lidar_history = []
        self.current_alt_lidar = 100.0
        self.lidar_received = False
        self.bridge = CvBridge()

        self.kf = KalmanFilter2D()
        self.last_tick_time = time.time()
        self.last_debug_time = time.time()
        self.landing_time = 0.0

        # --- MÉTRICAS ---
        self.t_start_phase2 = 0.0
        self.t_first_detection = 0.0
        self.first_detection_logged = False

        self.abort_lidar_count = 0
        self.abort_visual_count = 0

        self.boat_true_pose = None

        self.frames_without_detection_phase3 = 0

        self.blind_entry_vx = 0.0
        self.blind_entry_vy = 0.0
        self._blind_entry_logged = False

        self.lidar_std_at_phase4 = 0.0

        self.phase_history = []
        self.last_phase = -1
        # ----------------

        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.fx, self.fy, self.cx, self.cy = None, None, None, None
        self.err_x_m, self.err_y_m = 0.0, 0.0
        self.target_visible = False
        self.last_detection_time = 0.0

        self.state_sub = self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.lidar_sub = self.create_subscription(LaserScan, '/drone/range/lidar_down', self._lidar_cb, qos)
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self._image_cb, qos)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, qos)
        self.boat_odom_sub = self.create_subscription(Odometry, '/wamv/odom', self._boat_odom_cb, 10)

        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')

        self.thruster_left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.thruster_right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.boat_thrust_msg = Float64()
        self.boat_thrust_msg.data = 90.0

        self.sp = PoseStamped()
        self.sp.header.frame_id = 'map'
        self.phase = 0
        self.offboard_counter = 0
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)

        print(f"[INIT] Alinhamento Cêntrico Absoluto | MODO BARCO: {self.boat_mode}")

    def _state_cb(self, msg): self.state = msg
    def _pose_cb(self, msg): self.current_pose = msg
    def _boat_odom_cb(self, msg): self.boat_true_pose = msg

    def _lidar_cb(self, msg: LaserScan):
        if len(msg.ranges) > 0:
            mid_idx = len(msg.ranges) // 2
            distance = msg.ranges[mid_idx]
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                self.lidar_history.append(distance)
                if len(self.lidar_history) > 5:
                    self.lidar_history.pop(0)
                self.current_alt_lidar = float(np.median(self.lidar_history))
                self.lidar_received = True

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

    def _get_yaw(self):
        if not self.current_pose: return 0.0
        q = self.current_pose.pose.orientation
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            enhanced = cv2.convertScaleAbs(gray, alpha=1.6, beta=30)
            corners, ids, _ = cv2.aruco.detectMarkers(enhanced, self.aruco_dict, parameters=self.aruco_params)

            if self.fx is not None and self.cx is not None:
                cv2.line(cv_img, (int(self.cx), 0), (int(self.cx), cv_img.shape[0]), (255, 0, 0), 2)

                if ids is not None and self.current_pose is not None:
                    t_cx = np.mean(corners[0][0][:, 0])
                    t_cy = np.mean(corners[0][0][:, 1])
                    alt = self.current_alt_lidar if self.current_alt_lidar < 50 else self.search_alt

                    self.err_x_m = (t_cx - self.cx) * (alt / self.fx)
                    self.err_y_m = (t_cy - self.cy) * (alt / self.fy)

                    yaw = self._get_yaw()
                    body_x, body_y = -self.err_y_m, -self.err_x_m
                    world_err_x = body_x * math.cos(yaw) - body_y * math.sin(yaw)
                    world_err_y = body_x * math.sin(yaw) + body_y * math.cos(yaw)

                    target_world_x = self.current_pose.pose.position.x + world_err_x
                    target_world_y = self.current_pose.pose.position.y + world_err_y

                    self.kf.update(np.array([[target_world_x], [target_world_y]]))

                    if not self.first_detection_logged:
                        self.t_first_detection = time.time()
                        self.first_detection_logged = True

                    self.target_visible = True
                    self.last_detection_time = time.time()
                    cv2.aruco.drawDetectedMarkers(cv_img, corners, ids, (0, 255, 0))
                else:
                    self.target_visible = False

            if self.kf.is_initialized:
                cv2.putText(cv_img, f"KF VX:{self.kf.x[2,0]:.2f} VY:{self.kf.x[3,0]:.2f}",
                            (10, 30), 1, 2, (0, 255, 255), 2)

            cv2.imshow("Drone View", cv_img)
            cv2.waitKey(1)
        except CvBridgeError: pass

    def _print_report(self, now, land_x, land_y, vx, vy):
        duration = now - self.t_start_phase2
        detection_to_landing = now - self.t_first_detection if self.first_detection_logged else -1.0
        total_phase_transitions = len(self.phase_history)

        if self.boat_true_pose is not None:
            boat_x = self.boat_true_pose.pose.pose.position.x
            boat_y = self.boat_true_pose.pose.pose.position.y

            qx = self.boat_true_pose.pose.pose.orientation.x
            qy = self.boat_true_pose.pose.pose.orientation.y
            qz = self.boat_true_pose.pose.pose.orientation.z
            qw = self.boat_true_pose.pose.pose.orientation.w

            dz = 1.835
            offset_x = 2.0 * (qx * qz + qw * qy) * dz
            offset_y = 2.0 * (qy * qz - qw * qx) * dz

            true_aruco_x = boat_x + offset_x
            true_aruco_y = boat_y + offset_y

            final_err_x = self.current_pose.pose.position.x - true_aruco_x
            final_err_y = self.current_pose.pose.position.y - true_aruco_y
        else:
            final_err_x = self.current_pose.pose.position.x - land_x
            final_err_y = self.current_pose.pose.position.y - land_y

        euclidean_err = math.hypot(final_err_x, final_err_y)

        print("\n" + "="*60)
        print("  RELATÓRIO DE ATERRAGEM")
        print("="*60)
        print(f"  Modo Barco                    : {'Movimento' if self.boat_mode == 2 else 'Parado'}")
        print(f"  Tempo de Missão (s)           : {duration:.2f}")
        print(f"  Tempo Deteção→Aterragem (s)   : {detection_to_landing:.2f}")
        print("------------------------------------------------------------")
        print(f"  Erro Euclidiano (m)           : {euclidean_err:.3f}")
        print(f"  Erro Eixo X (m)               : {final_err_x:.3f}")
        print(f"  Erro Eixo Y (m)               : {final_err_y:.3f}")
        print("------------------------------------------------------------")
        print(f"  Transições de Fase            : {total_phase_transitions}")
        print(f"  Sequência de Fases            : {self.phase_history}")
        print(f"  Aborts LIDAR                  : {self.abort_lidar_count}")
        print(f"  Aborts Visual                 : {self.abort_visual_count}")
        print("------------------------------------------------------------")
        print(f"  Frames cego Fase 3            : {self.frames_without_detection_phase3}")
        print(f"  STD LIDAR na entrada Fase 4   : {self.lidar_std_at_phase4:.3f}")
        print(f"  Altitude ao Cortar (m)        : {self.current_alt_lidar:.2f}")
        print("------------------------------------------------------------")
        print(f"  VX KF entrada blind phase     : {self.blind_entry_vx:.2f}")
        print(f"  VY KF entrada blind phase     : {self.blind_entry_vy:.2f}")
        print(f"  Alvo Visível no Fim?          : {self.target_visible}")
        print("="*60)
        print("\n  [CSV] modo,t_missao,t_det_land,err_euc,err_x,err_y,transicoes,abort_lidar,abort_visual,frames_cego,std_lidar_f4,alt_corte,vx_blind,vy_blind")
        print(f"  [CSV] {self.boat_mode},{duration:.2f},{detection_to_landing:.2f},{euclidean_err:.3f},{final_err_x:.3f},{final_err_y:.3f},{total_phase_transitions},{self.abort_lidar_count},{self.abort_visual_count},{self.frames_without_detection_phase3},{self.lidar_std_at_phase4:.3f},{self.current_alt_lidar:.2f},{self.blind_entry_vx:.2f},{self.blind_entry_vy:.2f}")
        print("="*60 + "\n")

    def _tick(self):
        if not self.current_pose: return
        now = time.time()
        dt = now - self.last_tick_time
        self.last_tick_time = now

        self.kf.predict(dt)
        self.sp.header.stamp = self.get_clock().now().to_msg()

        vx = self.kf.x[2, 0] if self.kf.is_initialized else 0.0
        vy = self.kf.x[3, 0] if self.kf.is_initialized else 0.0

        land_x = self.kf.x[0, 0] if self.kf.is_initialized else self.current_pose.pose.position.x
        land_y = self.kf.x[1, 0] if self.kf.is_initialized else self.current_pose.pose.position.y

        pred_x = land_x + vx * self.lead_time
        pred_y = land_y + vy * self.lead_time

        # Registar transições de fase
        if self.phase != self.last_phase:
            self.phase_history.append(self.phase)
            self.last_phase = self.phase

        if now - self.last_debug_time >= 1.0:
            std_lidar = np.std(self.lidar_history) if len(self.lidar_history) > 0 else 0.0
            dist = math.hypot(land_x - self.current_pose.pose.position.x,
                              land_y - self.current_pose.pose.position.y) if self.kf.is_initialized else 0.0
            print(f"[DEBUG] FASE: {self.phase} | ALT: {self.current_alt_lidar:.2f}m | STD: {std_lidar:.2f} | VISÃO: {self.target_visible} | VX: {vx:.2f} VY: {vy:.2f} | DIST: {dist:.2f}m")
            self.last_debug_time = now

        if self.phase >= 2:
            self.boat_thrust_msg.data = 90.0 if self.boat_mode == 2 else 0.0
            self.thruster_left_pub.publish(self.boat_thrust_msg)
            self.thruster_right_pub.publish(self.boat_thrust_msg)

        if self.phase == 0:
            self.sp.pose.position.x = self.current_pose.pose.position.x
            self.sp.pose.position.y = self.current_pose.pose.position.y
            self.sp.pose.position.z = self.search_alt
            self.boat_thrust_msg.data = 7.0
            self.thruster_left_pub.publish(self.boat_thrust_msg)
            self.thruster_right_pub.publish(self.boat_thrust_msg)
            self.offboard_counter += 1
 
            if self.offboard_counter > 20:
                if self.state.mode != 'OFFBOARD':
                    self.mode_cli.call_async(SetMode.Request(custom_mode='OFFBOARD'))
                    self.boat_thrust_msg.data = 0.0
                    self.thruster_left_pub.publish(self.boat_thrust_msg)
                    self.thruster_right_pub.publish(self.boat_thrust_msg)
                if not self.state.armed:
                    self.arm_cli.call_async(CommandBool.Request(value=True))
                if self.state.mode == 'OFFBOARD' and self.state.armed:
                    print("[PHASE] OFFBOARD Confirmado.")
                    self.phase = 1

        elif self.phase == 1:
            self.sp.pose.position.x = self.current_pose.pose.position.x
            self.sp.pose.position.y = self.current_pose.pose.position.y
            self.sp.pose.position.z = self.search_alt
            self.sp.pose.position.x, self.sp.pose.position.y = self.target_gps_xy

            dist = math.hypot(self.target_gps_xy[0] - self.current_pose.pose.position.x,
                              self.target_gps_xy[1] - self.current_pose.pose.position.y)
            if dist < 0.5:
                print("[PHASE] GPS atingido. Iniciando Perseguição e Descida Coarse.")
                self.t_start_phase2 = time.time()
                self.phase = 2

        elif self.phase == 2:
            if self.kf.is_initialized and (now - self.last_detection_time) < 2.0:
                kf_err_x = pred_x - self.current_pose.pose.position.x
                kf_err_y = pred_y - self.current_pose.pose.position.y

                self.sp.pose.position.x = self.current_pose.pose.position.x + max(min(kf_err_x * 1.0, 0.8), -0.8)
                self.sp.pose.position.y = self.current_pose.pose.position.y + max(min(kf_err_y * 1.0, 0.8), -0.8)

                real_dist = math.hypot(land_x - self.current_pose.pose.position.x,
                                       land_y - self.current_pose.pose.position.y)

                if real_dist < 2.0:
                    self.sp.pose.position.z = max(self.current_pose.pose.position.z - 0.15, self.coarse_alt)
                else:
                    self.sp.pose.position.z = self.search_alt

                if (real_dist < self.precision_threshold and self.current_pose.pose.position.z < (self.coarse_alt + 0.5)) or \
                   (real_dist < 0.3 and self.current_alt_lidar < 1.5):
                    self.phase = 3
                    print(f"[PHASE] Aproximação Coarse concluída. Iniciando aterragem final.")

            else:
                self.sp.pose.position.x = self.current_pose.pose.position.x
                self.sp.pose.position.y = self.current_pose.pose.position.y
                if self.current_alt_lidar < 2.0:
                    print(f"[PHASE 2 RECOVERY] Visão perdida a {self.current_alt_lidar:.2f}m. A subir.")
                    self.sp.pose.position.z = self.search_alt
                else:
                    self.sp.pose.position.z = self.current_pose.pose.position.z

        elif self.phase == 3:
            if len(self.lidar_history) == 5 and np.std(self.lidar_history) > 0.5:
                print(f"[ABORT LIDAR] STD = {np.std(self.lidar_history):.2f}. Revertendo para Fase 2.")
                self.sp.pose.position.z = self.search_alt
                self.abort_lidar_count += 1
                self.phase = 2
                return

            has_vision = self.target_visible and (now - self.last_detection_time) < 0.5
            is_close = self.current_alt_lidar < self.blind_landing_alt

            if not has_vision:
                self.frames_without_detection_phase3 += 1

            if is_close or not has_vision:
                if not self._blind_entry_logged:
                    self.blind_entry_vx = vx
                    self.blind_entry_vy = vy
                    self._blind_entry_logged = True

                blind_lead = 1.5
                self.sp.pose.position.x = land_x + vx * blind_lead
                self.sp.pose.position.y = land_y + vy * blind_lead
                self.sp.pose.position.z = self.current_pose.pose.position.z - 0.25
            else:
                kf_err_x = pred_x - self.current_pose.pose.position.x
                kf_err_y = pred_y - self.current_pose.pose.position.y

                self.sp.pose.position.x = self.current_pose.pose.position.x + max(min(kf_err_x * 1.0, 0.8), -0.8)
                self.sp.pose.position.y = self.current_pose.pose.position.y + max(min(kf_err_y * 1.0, 0.8), -0.8)

                real_dist = math.hypot(land_x - self.current_pose.pose.position.x,
                                       land_y - self.current_pose.pose.position.y)

                if real_dist < 1.0:
                    self.sp.pose.position.z = self.current_pose.pose.position.z - 0.15
                else:
                    print(f"[ABORT VISUAL] Distância ({real_dist:.2f}m) insustentável. Revertendo para Fase 2.")
                    self.sp.pose.position.z = self.search_alt
                    self.abort_visual_count += 1
                    self._blind_entry_logged = False
                    self.phase = 2
                    return

            if self.lidar_received and self.current_alt_lidar < self.landing_threshold_alt:
                self.lidar_std_at_phase4 = np.std(self.lidar_history) if self.lidar_history else 0.0
                print(f"[FASE 4] Descida Forçada (Plunge): Sincronização vertical agressiva.")
                self.landing_time = now
                self.phase = 4

        elif self.phase == 4:
            self.sp.pose.position.x = pred_x
            self.sp.pose.position.y = pred_y
            self.sp.pose.position.z = self.current_pose.pose.position.z - 2.0

            if now - self.landing_time > 1.5:
                self._print_report(now, land_x, land_y, vx, vy)

                print("[FINAL] Ancoragem concluída. Executando MAVLink Force Disarm.")
                req = CommandLong.Request()
                req.command = 400
                req.param1 = 0.0
                req.param2 = 21196.0
                self.cmd_cli.call_async(req)
                self.mode_cli.call_async(SetMode.Request(custom_mode='MANUAL'))
                self.phase = 5

        elif self.phase == 5:
            pass

        self.sp_pub.publish(self.sp)


def main():
    rclpy.init()
    node = AutonomousLandingFinal()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()