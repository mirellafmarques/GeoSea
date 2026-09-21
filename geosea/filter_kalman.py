"""Extended Kalman Filter for Hydrographic Navigation"""

import numpy as np
import time, traceback
from dataclasses import dataclass
import threading
from qgis.PyQt.QtCore import QObject, pyqtSignal, QMutex, QMutexLocker
from scipy.stats import chi2

# Pacotes de Dados
@dataclass
class GNSSData:
    easting: float   # UTM 
    northing: float  # UTM 
    altitude: float
    speed: float
    accuracy: float
    fix_type: str = "GPS"  # GPS, DGPS, RTK_FLOAT, RTK_FIX, DR
    hdop: float = 1.0
    vdop: float = 1.0
    satellites: int = 0
    timestamp: float = 0.0
    cog: float = 0.0   

@dataclass
class IMUData:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    delta_t: float
    timestamp: float

@dataclass
class HeadingData:
    angle: float
    reliability: float
    timestamp: float

@dataclass
class FusionState:
    easting: float = 0.0
    northing: float = 0.0
    altitude: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    accuracy: float = 0.0
    mode: str = "ACQUIRING"

# 10-State Extended Kalman Filter
class KalmanFilter:
    """Extended Kalman Filter. State: [x, y, vx, vy, psi, b_ax, b_ay, b_w, vcx, vcy]. Where vcx, vcy = estimated tidal current (m/s),
vx = velocity relative to the water and vcx = current; Ground Velocity = Boat Velocity + Current."""
    def __init__(self):

        self.n_states = 10

        # Initial State
        self.x = np.zeros(self.n_states)
        self.P = np.eye(self.n_states) * 100  # Initial uncertainty
        self.F = np.eye(self.n_states) 
        self.H_gnss = np.zeros((2, self.n_states))  # Observations
        self.H_gnss[0, 0] = 1.0  # x
        self.H_gnss[1, 1] = 1.0  # y
        self.H_heading = np.zeros((1, self.n_states)) 
        self.H_heading[0, 4] = 1.0  
        self.H_velocity = np.zeros((2, self.n_states))
        self.H_velocity[0,2] = 1.0   # vx
        self.H_velocity[1,3] = 1.0   # vy
        self.H_velocity[0,8]=1
        self.H_velocity[1,9]=1

        #  Q-Matrix
        self.Q_base = np.zeros((self.n_states, self.n_states))
        self.Q_base[2:4, 2:4] = np.eye(2) * 5.0     # vx, vy
        self.Q_base[4, 4] = 0.05                    # Heading
        self.Q_base[5:7, 5:7] = np.eye(2) * 0.0001  # Accelerometer bias (random walk)
        self.Q_base[7, 7] = 0.0001                  # Gyroscope bias
        self.Q_base[8:10,8:10] = np.eye(2) * 5e-5
        self.R_gnss = np.eye(2) * 25  # Measurement noise
        self.R_heading = np.eye(1) * 5 
        self.R_velocity = np.eye(2) * 0.25 # GNSS velocity noise (m/s)
        self.chi2_threshold = chi2.ppf(0.95, df=2) # 95% confidence, 2 DOF for GNSS
        self.heading_chi2_threshold = chi2.ppf(0.95, df=1) # 95% confidence, 1 DOF
        self.velocity_chi2_threshold = chi2.ppf(0.95, df=2)

        # Statistic
        self.gnss_accepted = 0
        self.gnss_rejected = 0
        self.heading_accepted = 0
        self.heading_rejected = 0
        self.velocity_accepted = 0
        self.velocity_rejected = 0

        self.last_time = None
        self.mode = "ACQUIRING"
        self.last_dt = 0.0
      
        self.last_innovation = np.zeros(2)
        self.last_nis = 0.0

        self.last_heading_innovation = 0.0
        self.last_heading_nis = 0.0
 
        self.last_velocity_innovation = np.zeros(2)
        self.last_velocity_nis = 0.0
          
            
    def predict(self, dt: float, accel_meas: np.ndarray = None, gyro_meas: np.ndarray = None, has_imu: bool = True):
        if dt <= 0 or dt > 2.0:
            return self.x
        self.last_dt = dt

        x, y, vx, vy, psi = self.x[0:5]
        b_ax, b_ay, b_w = self.x[5:8]
        vcx, vcy = self.x[8:10]

        # Detects the motion sensor (IMU/MRU)
        if has_imu and accel_meas is not None and gyro_meas is not None:
            ax_corrected = accel_meas[0] - b_ax
            ay_corrected = accel_meas[1] - b_ay
            wz_corrected = gyro_meas[2] - b_w 
        else:
            ax_corrected = 0.0
            ay_corrected = 0.0
            wz_corrected = 0.0

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        ax_global = ax_corrected * cos_psi - ay_corrected * sin_psi
        ay_global = ax_corrected * sin_psi + ay_corrected * cos_psi

        # Limits acceleration
        MAX_ACCEL_FILTER = 1.5
        ax_global = np.clip(ax_global, -MAX_ACCEL_FILTER, MAX_ACCEL_FILTER)
        ay_global = np.clip(ay_global, -MAX_ACCEL_FILTER, MAX_ACCEL_FILTER)

        # Predictions
        x_pred = x + (vx + vcx) * dt
        y_pred = y + (vy + vcy) * dt
        vx_pred = vx + ax_global * dt
        vy_pred = vy + ay_global * dt
        
        speed = np.sqrt(vx_pred**2 + vy_pred**2)
        if speed < 0.3:
            vx_pred = 0.0
            vy_pred = 0.0
            speed = 0.0

        MAX_SPEED = 5.0 
        if speed > MAX_SPEED:
            vx_pred *= MAX_SPEED / speed
            vy_pred *= MAX_SPEED / speed
        psi_pred = psi + wz_corrected * dt 
        
        b_ax_pred, b_ay_pred, b_w_pred = b_ax, b_ay, b_w

        # 1st-order Gauss-Markov
        self.current_decay = 0.9995
        vcx_pred = self.current_decay * vcx
        vcy_pred = self.current_decay * vcy

        MAX_CURRENT_SPEED = 2.0  # m/s
        current_speed = np.hypot( vcx_pred, vcy_pred )
        if current_speed > MAX_CURRENT_SPEED:
            fator = MAX_CURRENT_SPEED / current_speed
            vcx_pred *= fator
            vcy_pred *= fator
        
        self.x = np.array([x_pred, 
                           y_pred, 
                           vx_pred, 
                           vy_pred, 
                           psi_pred, 
                           b_ax_pred, 
                           b_ay_pred, 
                           b_w_pred, 
                           vcx_pred, 
                           vcy_pred])
                           
        self.x[4] = self.x[4] % (2 * np.pi)
        if self.x[4] > np.pi:
            self.x[4] -= 2 * np.pi
            
        # JACOBIANA F
        self.F = np.eye(self.n_states)
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.F[0, 8] = dt
        self.F[1, 9] = dt
        self.F[2, 2] = 1.0
        self.F[3, 3] = 1.0
        self.F[2, 4] = (-ax_corrected * sin_psi - ay_corrected * cos_psi) * dt
        self.F[3, 4] = (ax_corrected * cos_psi - ay_corrected * sin_psi) * dt

        # # Only calculate IMU/MRU and bias derivatives if the sensor is active
        if has_imu:
            self.F[2, 5] = -np.cos(psi) * dt
            self.F[2, 6] = np.sin(psi) * dt
            self.F[3, 5] = -np.sin(psi) * dt
            self.F[3, 6] = -np.cos(psi) * dt
            self.F[4, 7] = -dt
            
        self.F[8, 8] = self.current_decay # Current
        self.F[9, 9] = self.current_decay

        # Q-Matrix
        qa = self.Q_base[2,2]
        Qpv = qa * np.array([ [dt**4/4, dt**3/2], [dt**3/2, dt**2] ])
        Q = np.zeros((self.n_states, self.n_states))
        # East
        Q[np.ix_([0,2],[0,2])] = Qpv
        # North
        Q[np.ix_([1,3],[1,3])] = Qpv
        # Heading
        Q[4,4] = self.Q_base[4,4] * dt
        # Accelerometer bias
        Q[5:7,5:7] = self.Q_base[5:7,5:7] * dt
        # Gyroscope bias
        Q[7,7] = self.Q_base[7,7] * dt
        # Current
        Q[8:10,8:10] = self.Q_base[8:10,8:10] * dt
        self.Q = Q

        # Covariance Update
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)
    
        if not np.all(np.isfinite(self.P)):
            return self.x
        eig = np.linalg.eigvalsh(self.P)
        if eig.min() < 0:
            self.P += np.eye(self.n_states) * (-eig.min() + 1e-9)
        return self.x
            
    def correct_velocity(self, velocity: np.ndarray, accuracy: float = 0.5):
        if not np.all(np.isfinite(velocity)):
            return
        accuracy = max(float(accuracy), 0.30)
        self.R_velocity = np.eye(2) * (accuracy ** 2)
        H = np.zeros((2, self.n_states))
        H[:,2:4] = np.eye(2)      # vx vy
        H[:,8:10] = np.eye(2)     # vcx vcy
        predicted_velocity = self.x[2:4] + self.x[8:10]
        innovation = velocity - predicted_velocity
        S = H @ self.P @ H.T + self.R_velocity

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        nis = innovation.T @ S_inv @ innovation
        self.last_velocity_innovation = innovation.copy()
        self.last_velocity_nis = float(nis)

        if nis > self.velocity_chi2_threshold:
            self.velocity_rejected += 1
            return
        self.velocity_accepted += 1
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        if not np.all(np.isfinite(self.x)):
            return

        I = np.eye(self.n_states)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ self.R_velocity @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        speed = np.linalg.norm(self.x[2:4])
        self.x[4] = np.arctan2(np.sin(self.x[4]), np.cos(self.x[4]) )

  
    
    def correct_gnss(self, position: np.ndarray, accuracy: float, fix_type: str = "GPS"):
        fix_multipliers = {
            'RTK_FIX': 0.05,     # 5% da accuracy
            'RTK_FLOAT': 0.3,    # 30%
            'DGPS': 0.7,         # 70%
            'GPS': 1.0,          # 100%
            'DR': 3.0            # 300% (dead reckoning)
        }
        
        multiplier = fix_multipliers.get(fix_type, 1.0)
        adjusted_accuracy = accuracy * multiplier
        adjusted_accuracy = max(adjusted_accuracy, 0.5)  
        adjusted_accuracy = min(adjusted_accuracy, 50.0)  
        
        self.R_gnss = np.eye(2) * (adjusted_accuracy ** 2)
        z = position # [x, y] measured
        z_pred = self.x[0:2]  # [x, y] predicted
        innovation = z - z_pred

        H = self.H_gnss
        S = H @ self.P @ H.T + self.R_gnss

        # Mahalanobis distance
        try:
            S_inv = np.linalg.inv(S) 
            nis = float(innovation.T @ S_inv @ innovation)
            self.last_innovation = innovation.copy()
            self.last_nis = nis
            self.last_innovation = innovation.copy()
            self.last_nis = float(nis)
        except np.linalg.LinAlgError:
            print("[EKF]")
            return
        
        if not np.isfinite(nis):
            self.gnss_rejected += 1
            return

        if nis > self.chi2_threshold:
            self.gnss_rejected += 1

            # If the prediction deviates from the actual GNSS position, it limits the vessel's drift
            if np.linalg.norm(innovation) > 20.0:
                self.x[0] = position[0]
                self.x[1] = position[1]
                # Reduces the estimated speed to prevent the drift from continuing in the next prediction
                self.x[2] *= 0.5
                self.x[3] *= 0.5
                self.P[0, 0] = max(self.P[0, 0], 25.0)
                self.P[1, 1] = max(self.P[1, 1], 25.0)
            return
        
        self.gnss_accepted += 1

        K = self.P @ H.T @ S_inv # Kalman gain
        self.x = self.x + K @ innovation # Updates the state
        if not np.all(np.isfinite(self.x)):
            return

        # Updates the covariance
        I_KH = np.eye(self.n_states) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R_gnss @ K.T
        if not np.all(np.isfinite(self.P)):
            return
        
        # Normalizes the heading
        self.x[4] = self.x[4] % (2 * np.pi)
        if self.x[4] > np.pi:
            self.x[4] -= 2 * np.pi
            
        self.mode = f"GNSS_{fix_type}"
        # Statistical log every 100 corrections
        total = self.gnss_accepted + self.gnss_rejected
        if total % 100 == 0:
            print(f" [EKF STATS] GNSS: {self.gnss_accepted} accepted, "
                  f"{self.gnss_rejected} rejected "
                  f"({self.gnss_accepted/total*100:.1f}% acceptation)")
        
        
    def correct_heading(self, heading: float, reliability: float):
        """Corrects the EKF heading using a direct angular measurement."""
        if not np.isfinite(heading):
            return
        
        z = np.arctan2( np.sin(heading), np.cos(heading) ) # Normalizes the measurement
        z_pred = self.x[4] 
        y = z - z_pred # Angular innovation
        y = np.arctan2(np.sin(y), np.cos(y) )
        self.last_heading_innovation = float(y)

        reliability = float( np.clip(reliability, 0.1, 1.0))
        sigma_deg = 2.0 / reliability
        sigma_rad = np.radians(np.clip(sigma_deg, 2.0, 15.0))
        self.R_heading = np.array([[sigma_rad ** 2]])
        H = np.zeros((1, self.n_states))
        H[0, 4] = 1.0
        S = H @ self.P @ H.T + self.R_heading
        if not np.isfinite(S[0, 0]) or S[0, 0] <= 0:
            return
        nis = (y ** 2) / S[0, 0]
        self.last_heading_nis = float(nis)
        if nis > self.heading_chi2_threshold:
            self.heading_rejected += 1
            return

        self.heading_accepted += 1

        # Gain
        K = self.P @ H.T / S[0, 0]
        # Update
        self.x = self.x + K.flatten() * y
        if not np.all(np.isfinite(self.x)):
            return
        # Covariance
        I_KH = (np.eye(self.n_states)- K @ H )
        self.P = (I_KH @ self.P @ I_KH.T + K @ self.R_heading @ K.T )
        self.P = 0.5 * ( self.P + self.P.T )
        # Normalization
        self.x[4] = np.arctan2(np.sin(self.x[4]), np.cos(self.x[4]) )


    def get_state(self) -> np.ndarray:
        return self.x.copy()
        
    def get_biases(self):
        return {'b_ax': self.x[5],
            'b_ay': self.x[6],
            'b_w': self.x[7],
            'vcx': self.x[8],
            'vcy': self.x[9]}
    

    def get_current_estimate(self):
        return {'vcx': self.x[8],
            'vcy': self.x[9],
            'speed': np.sqrt(self.x[8]**2 + self.x[9]**2),
            'direction': np.degrees(np.arctan2(self.x[8], self.x[9])) % 360 }

    def get_mode(self) -> str:
        return self.mode

    def reset(self):
        self.x = np.zeros(self.n_states)
        self.P = np.eye(self.n_states) * 100
        self.F = np.eye(self.n_states)
        self.mode = "ACQUIRING"
        self.gnss_accepted = 0
        self.gnss_rejected = 0
        self.heading_accepted = 0
        self.heading_rejected = 0
        self.velocity_accepted = 0
        self.velocity_rejected = 0
        self.last_innovation = np.zeros(2)
        self.last_nis = 0.0
        self.last_velocity_innovation = np.zeros(2)
        self.last_velocity_nis = 0.0
        self.last_heading_innovation = 0.0
        self.last_heading_nis = 0.0
        self.last_dt = 0.0

    def get_accuracy(self):
        """Returns the horizontal precision (1σ)."""
        p = 0.5 * (self.P + self.P.T)      
        sigma2 = p[0,0] + p[1,1]
        if not np.isfinite(sigma2):
            return np.nan

        sigma2 = max(sigma2, 0.0)
        return float(np.sqrt(sigma2))


class FusionEngine(QObject):
    
    state_updated = pyqtSignal(object) 
    debug_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.kalman = KalmanFilter()
        self.current_state = FusionState()
        self.mutex = QMutex() # O cadeado de segurança
        self.last_gnss_time = 0
        self.last_imu_time = 0
        self.last_heading_time = 0
        self.is_initialized = False

        self.last_valid_heading = 0.0 
        self.heading_frozen = False     
        self.min_speed_for_heading = 0.5  # m/s (~1 knot)

        self.has_imu = False
        self.has_heading = False
        self.heading_timeout = 2.0  # Timeout for considering heading lost

        self.total_updates = 0
        self.updates_with_imu = 0
        self.updates_with_heading = 0


    def check_sensors_availability(self):
        """Checks for the availability of motion and heading sensors."""
        current_time = time.time()        
        if self.last_imu_time > 0:
            self.has_imu = (current_time - self.last_imu_time) < 0.5        
        if self.last_heading_time > 0:
            self.has_heading = (current_time - self.last_heading_time) < self.heading_timeout
        else:
            self.has_heading = False


    def process_imu(self, imu_data: IMUData):
        """Processes the motion sensor data using the filter prediction."""
        try:
            locker = QMutexLocker(self.mutex)
            self.last_imu_time = imu_data.timestamp
            self.check_sensors_availability()
            if not self.is_initialized or not self.has_imu:  # Only process IMU if initial GNSS is already available
                return
            # Limits acceleration
            MAX_ACCEL = 20.0  # m/s²
            accel_x = np.clip(imu_data.accel_x, -MAX_ACCEL, MAX_ACCEL)
            accel_y = np.clip(imu_data.accel_y, -MAX_ACCEL, MAX_ACCEL)
            # Limits the gyro
            MAX_GYRO = np.pi  # rad/s
            gyro_z = np.clip(imu_data.gyro_z, -MAX_GYRO, MAX_GYRO)
            accel_meas = np.array([accel_x, accel_y])
            gyro_meas = np.array([imu_data.gyro_x, imu_data.gyro_y, gyro_z])
            # Prediction
            state = self.kalman.predict(imu_data.delta_t, accel_meas, gyro_meas, has_imu=True)
            self._update_state_from_kalman(state)
            self.updates_with_imu += 1
        except Exception as e:
            traceback.print_exc()

    
    def process_gnss(self, gnss_data: GNSSData):
        """Processes GNSS with statistical validation."""
        if np.isnan(gnss_data.easting) or np.isnan(gnss_data.northing):
            return
        try:
            locker = QMutexLocker(self.mutex)
    
            # Cold start
            if not self.is_initialized:
                self.kalman.x[0] = gnss_data.easting
                self.kalman.x[1] = gnss_data.northing
                speed_ms = getattr(gnss_data, 'speed', 0.0) * 0.514444
                cog_val = getattr(gnss_data, "cog", 0.0)
                if cog_val is None or not np.isfinite(cog_val):
                    cog_val = 0.0   
                cog_rad = np.radians(cog_val)
                cog_deg = (getattr(gnss_data, "cog", 0.0))
                if cog_deg is None or not np.isfinite(cog_deg):
                    cog_deg = 0.0

                cog = np.radians(cog_deg)
                self.kalman.x[2] = speed_ms * np.sin(cog_rad)
                self.kalman.x[3] = speed_ms * np.cos(cog_rad)
                self.kalman.x[4] = cog_rad
                self.last_gnss_time = gnss_data.timestamp
                self.is_initialized = True
                self._update_state_from_kalman(self.kalman.get_state())
                return

            dt = gnss_data.timestamp - self.last_gnss_time
            if dt <= 0:
                return

            if dt >= 3.0:
                self.is_initialized = False
                self.debug_signal.emit(f"Significant time jump ({dt:.1f}s). Restarting the EKF.")
                return
            
            if not self.has_imu:
                self.kalman.predict(dt, has_imu=False)

            # GNSS Correction
            position = np.array([gnss_data.easting, gnss_data.northing])
            accuracy = max(getattr(gnss_data, 'accuracy', 2.0), 2.0)
            fix_type = getattr(gnss_data, 'fix_type', 'GPS')
            self.kalman.correct_gnss(position, accuracy, fix_type)
            speed_ms = getattr(gnss_data, 'speed', 0.0) * 0.514444
            cog = np.radians(getattr(gnss_data, "cog", 0.0))
            vx = speed_ms * np.sin(cog)
            vy = speed_ms * np.cos(cog)

            # Detects the stationary vessel
            STOP_SPEED = 0.20  # m/s ~ 0,39 kt
            if speed_ms < STOP_SPEED:
                self.kalman.x[2] = 0.0
                self.kalman.x[3] = 0.0
                self.kalman.x[8] = 0.0
                self.kalman.x[9] = 0.0

            velocity = np.array([vx, vy])
            velocity_accuracy = {
                "RTK_FIX":0.05,
                "RTK_FLOAT":0.10,
                "DGPS":0.20,
                "GPS":0.50,
                "DR":1.50 }

            sigma_v = max(velocity_accuracy.get(fix_type, 0.5), 0.30)
            self.kalman.correct_velocity(velocity, sigma_v)
            state = self.kalman.get_state()
            vxg = state[2] + state[8]
            vyg = state[3] + state[9]

            speed = np.hypot(vxg, vyg)
            if speed_ms > 1.0:
                cog = getattr(gnss_data, 'cog', None)
                if cog is not None and not self.has_heading:
                    self.kalman.correct_heading(np.radians(cog), 0.3)

            self.last_gnss_time = gnss_data.timestamp
            self.total_updates += 1
            self._update_state_from_kalman(self.kalman.get_state())

        except Exception as e:
            traceback.print_exc()
            
            
    def process_heading(self, heading_data: HeadingData):
        """Processes the dedicated heading sensor."""
        try:
            locker = QMutexLocker(self.mutex)
            self.last_heading_time = heading_data.timestamp
            self.check_sensors_availability()
            if not self.is_initialized:
                return
            heading_rad = np.radians(heading_data.angle) 
            reliability = np.clip(heading_data.reliability, 0.1, 0.99)

            if self.current_state.speed < 0.3:
                reliability *= 0.5  
            self.kalman.correct_heading(heading_rad, reliability)
            self.updates_with_heading += 1
            self._update_state_from_kalman(self.kalman.get_state())

        except Exception as e:
            self.debug_signal.emit(f"ERROR: {str(e)}")
           

    def _update_state_from_kalman(self, state: np.ndarray):
        try:
            self.current_state.easting = float(state[0])
            self.current_state.northing = float(state[1])

            vx_ground = state[2] + state[8]
            vy_ground = state[3] + state[9]
            self.current_state.speed = float( np.hypot(vx_ground, vy_ground) )
            self.current_state.heading = float(np.degrees(state[4])) % 360  # Normalizado
            self.current_state.accuracy = self.kalman.get_accuracy()
            self.current_state.mode = self.kalman.get_mode()

            raw_heading = float(np.degrees(state[4])) % 360         
            if self.has_heading:
                display_heading = raw_heading
                self.last_valid_heading = display_heading
                self.heading_frozen = False
                
            elif self.current_state.speed > self.min_speed_for_heading:
                # Sensorless but in motion, using GNSS COG
                display_heading = raw_heading
                self.last_valid_heading = display_heading
                self.heading_frozen = False
            else:
                # No sensor and stationary: uses the last valid heading
                if not self.heading_frozen:
                    self.heading_frozen = True
                display_heading = self.last_valid_heading

            self.current_state.heading = display_heading    
            self.state_updated.emit(self.current_state) 
        except Exception as e:
            traceback.print_exc()


    def get_fusion_state(self) -> FusionState:
        locker = QMutexLocker(self.mutex)
        return self.current_state

    def get_stats(self):
        """EKF statistics."""
        return {
            'gnss_accepted': self.kalman.gnss_accepted,
            'gnss_rejected': self.kalman.gnss_rejected,
            'heading_accepted': self.kalman.heading_accepted,
            'heading_rejected': self.kalman.heading_rejected,
            'has_imu': self.has_imu,
            'has_heading': self.has_heading,
            'current_estimate': self.kalman.get_current_estimate(),
            'biases': self.kalman.get_biases() }
    

    def get_current_state(self) -> FusionState:
        locker = QMutexLocker(self.mutex)
        return self.current_state    

        
    def reset_filter(self):
        locker = QMutexLocker(self.mutex)
        self.kalman.reset()
        self.current_state = FusionState()
        self.last_gnss_time = 0
        self.last_imu_time = 0
        self.last_heading_time = 0
        self.is_initialized = False
        self.total_updates = 0
        self.updates_with_imu = 0
        self.updates_with_heading = 0
        self.state_updated.emit(self.current_state)
        self.debug_signal.emit("Filtro Kalman resetado")


    def get_qa_metrics(self):
        """Returns EKF state data and metrics to Quality Analysis (QA)."""
        locker = QMutexLocker(self.mutex)
        return {
            "pos": np.array([self.current_state.easting, self.current_state.northing]),
            "heading": self.current_state.heading,
            "accuracy": self.current_state.accuracy,
            "mode": self.current_state.mode,
            
            "covariance": self.kalman.P.copy(),
            "innovation": getattr(self.kalman, 'last_innovation', None),
            "nis": getattr(self.kalman, 'last_nis', None),
            "position_innovation": getattr(self.kalman, "last_innovation", None),
            "position_nis": getattr(self.kalman, "last_nis", None),

            "velocity_innovation": getattr(self.kalman, "last_velocity_innovation", None),
            "velocity_nis": getattr(self.kalman, "last_velocity_nis", None),
            "chi2_threshold": getattr(self.kalman, 'chi2_threshold', 6.0),
            "heading_innovation":getattr(self.kalman, "last_heading_innovation", None),
            "heading_nis": getattr(self.kalman, "last_heading_nis", None)   
        }
    
