from qgis.PyQt.QtCore import QObject, pyqtSignal, QIODevice
from qgis.PyQt.QtNetwork import QUdpSocket, QHostAddress
from qgis.PyQt.QtSerialPort import QSerialPort
from qgis.PyQt.QtCore import QObject, pyqtSignal, QIODevice, QTimer
from qgis.core import QgsMessageLog, Qgis
from dataclasses import dataclass
import time, re, struct

@dataclass
class IMUData:
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    roll: float   
    pitch: float  
    yaw: float    
    delta_t: float
    timestamp: float

class MRUManager(QObject):
    mru_data_updated = pyqtSignal(float, float, float, float) # roll, pitch, yaw, heave
    imu_data_for_fusion = pyqtSignal(object)  
    connection_status = pyqtSignal(bool, str)
    debug_signal = pyqtSignal(str)
    accel_bias_estimated = pyqtSignal(float, float) # ax_bias, ay_bias
    gyro_bias_estimated = pyqtSignal(float)         # gz_bias
        

    def __init__(self):
        super().__init__()
        self.serial = None
        self.udp_socket = None
        self.last_timestamp = None

        # Estimated FKE biases
        self.gyro_bias_z = 0.0   # Gyroscope bias
        self.accel_bias_x = 0.0  # Accelerometer bias X
        self.accel_bias_y = 0.0  # Accelerometer bias Y
        
        # Scale factor
        self.accel_scale = 1.0  # m/s² 
        self.gyro_scale = 1.0   # rad/s

        # Variables for the WitMotion WT901C sensor
        self._buffer = bytearray() # Text/Binary
        self._wit_accel = [0.0, 0.0, 0.0]
        self._wit_gyro = [0.0, 0.0, 0.0]
        self._wit_angle = [0.0, 0.0, 0.0]

        # Parsers
        self.parsers = {"$PASHR": self._parse_pashr,
            "$PRDID": self._parse_prdid,
            "$IMU": self._parse_imu }
             
    def connect_serial(self, port_name, baud_rate, data_bits):
        self.disconnect_mru(silencioso=True) 
        self.serial = QSerialPort(self)
        self.serial.setPortName(port_name)
        self.serial.setBaudRate(int(baud_rate))
        if self.serial.open(QIODevice.ReadOnly):
            self.debug_signal.emit("Port successfully opened.")
            self.serial.readyRead.connect(self.read_serial_data)
            self.connection_status.emit(True, f"Serial port opened: {port_name}")
            self.debug_signal.emit(f"Connected to {port_name}")
        else:
            erro = self.serial.errorString()
            self.connection_status.emit(False, f"Failed to open port: {erro}")

    def connect_network(self, ip, port):
        self.disconnect_mru()
        self.udp_socket = QUdpSocket(self)
        if self.udp_socket.bind(QHostAddress(ip), int(port)):
            self.udp_socket.readyRead.connect(self.read_udp_data)
            self.connection_status.emit(True, f"Connected to {ip}:{port}")
            self.debug_signal.emit(f"Connected")
        else:
            self.connection_status.emit(False, "communication failure")

    def disconnect_mru(self, silencioso=False):
        if self.serial and self.serial.isOpen():
            self.serial.close()
        if self.udp_socket:
            self.udp_socket.close()
        self.serial = None
        self.udp_socket = None
        if not silencioso:
            self.connection_status.emit(False, "Disconnected")

    def read_serial_data(self):
        if not self.serial:
            return
        raw = self.serial.readAll().data()
        QgsMessageLog.logMessage(f"{len(raw)} bytes received","GeoSea",
            Qgis.Info)
        if not raw:
            return
        self._buffer.extend(raw)
        self._process_buffer()

    def read_udp_data(self):
        while self.udp_socket.hasPendingDatagrams():
            datagram, _, _ = self.udp_socket.readDatagram(
                self.udp_socket.pendingDatagramSize())
            if datagram:
                self._buffer.extend(datagram)
        self._process_buffer()

    def _process_buffer(self):
            """Processes the observations."""
            while True:
                if not self._buffer:
                    return
    
                # Text
                if self._buffer[0] in (0x24, 0x40):      # $ ou @
                    pos = self._buffer.find(b'\n')
                    if pos < 0:
                        return
                    line = bytes(self._buffer[:pos + 1])
                    del self._buffer[:pos + 1]
                    try:
                        self.parse_motion_sensor(
                            line.decode("ascii", errors="ignore").strip() )
                    except Exception:
                        pass
                    continue
    
                # WitMotion Protocol
                idx = self._find_witmotion_header()
                if idx >= 0:
                    if idx > 0:
                        del self._buffer[:idx]
                    if len(self._buffer) < 11:
                        return
                    packet = bytes(self._buffer[:11])
                    if self._check_witmotion_checksum(packet):
                        del self._buffer[:11]
                        self.parse_motion_sensor(packet)
                    else:
                        del self._buffer[0]
                    continue
                self._buffer.clear()
                return            

    def _find_witmotion_header(self):
        valid_types = {0x51, 0x52, 0x53}

        for i in range(len(self._buffer) - 1):
            if self._buffer[i] == 0x55 and self._buffer[i + 1] in valid_types:
                return i
        return -1
            
    def _validate_nmea_checksum(self, sentence):
        """Checks the integrity of the NMEA 0183 sentence. Returns True if valid, or False if the packet is corrupted."""
        if not sentence.startswith('$') or '*' not in sentence:
            return True # Accepts CSV or TSS formats
        try:
            data_part, checksum_part = sentence.split('*', 1)
            chars_to_compute = data_part[1:]
            calculated_checksum = 0
            for char in chars_to_compute:
                calculated_checksum ^= ord(char)
            # Formats the result to hexadecimal
            calculated_hex = f"{calculated_checksum:02X}"
            return calculated_hex == checksum_part[:2].upper()
        except Exception:
            return False
                    
    def parse_motion_sensor(self, data):
        """Redirects the input to the corresponding decoder (Text or Binary)."""
        try:
            if isinstance(data, (bytes, bytearray)):
                if len(data) >= 11 and data[0] == 0x55:
                    self._parse_witmotion_packet(data)
                    return

                if not isinstance(data, str):
                    return
                data = data.strip()
                if not data:
                    return
                if data.startswith("$"):
                    if not self._validate_nmea_checksum(data):
                        return

                    header = data.split(",")[0]
                    parser = self.parsers.get(header)
                    if parser:
                        parser(data)
                        return
            self._parse_unstructured_data(data)
        except Exception as e:
            self.debug_signal.emit(str(e))

    def _parse_pashr(self, raw_string):
        """Decodes the $PASHR sentence (NMEA protocol)."""
        parts = raw_string.split(',')
        if len(parts) >= 7:
            try:
                yaw = float(parts[2])
                roll = float(parts[4])
                pitch = float(parts[5])
                heave = float(parts[6].split('*')[0])
                self.mru_data_updated.emit(roll, pitch, yaw, heave)
            except ValueError:
                return

    def _parse_prdid(self, raw_string):
        """Decodes the $PRDID sentence (NMEA protocol)."""
        parts = raw_string.split(',')
        if len(parts) >= 4:
            try:
                pitch = float(parts[1])
                roll = float(parts[2])
                yaw = float(parts[3].split('*')[0]) 
                heave = 0.0 
                self.mru_data_updated.emit(roll, pitch, yaw, heave)
            except ValueError:
                return

    def _parse_imu(self, raw_string):
        """Decodes the $IMU sentence."""
        parts = raw_string.split(',')
        if len(parts) >= 10:
            try:
                ax = float(parts[1]) * self.accel_scale
                ay = float(parts[2]) * self.accel_scale
                az = float(parts[3]) * self.accel_scale
                gx = float(parts[4]) * self.gyro_scale
                gy = float(parts[5]) * self.gyro_scale
                gz = float(parts[6]) * self.gyro_scale
                
                roll = float(parts[7])
                pitch = float(parts[8])
                yaw = float(parts[9].split('*')[0])
                heave = 0.0

                self.mru_data_updated.emit(roll, pitch, yaw, heave)
                self._process_imu_physics(ax, ay, az, gx, gy, gz, roll, pitch, yaw)
            except ValueError:
                return

    def _parse_unstructured_data(self, raw_string):
        """Processes CSV and TSS formats."""
        ax = ay = az = 0.0
        gx = gy = gz = 0.0
        roll = pitch = yaw = heave = 0.0
        has_physics_data = False

        try:
            # CSV - timestamp,ax,ay,az,gx,gy,gz
            if ',' in raw_string and not raw_string.startswith('$'):
                parts = raw_string.split(',')
                # Minimum of 6 elements
                if len(parts) >= 6:
                    offset = 1 if parts[0].replace('.', '').replace('-', '').isdigit() else 0
                    if len(parts) >= 6 + offset:
                        ax = float(parts[0+offset]) * self.accel_scale
                        ay = float(parts[1+offset]) * self.accel_scale
                        az = float(parts[2+offset]) * self.accel_scale
                        gx = float(parts[3+offset]) * self.gyro_scale
                        gy = float(parts[4+offset]) * self.gyro_scale
                        gz = float(parts[5+offset]) * self.gyro_scale
                        has_physics_data = True

            # TSS
            elif raw_string.startswith('@'):
                accel_match = re.search(r'@a\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', raw_string)
                if accel_match:
                    ax = float(accel_match.group(1)) * self.accel_scale
                    ay = float(accel_match.group(2)) * self.accel_scale
                    az = float(accel_match.group(3)) * self.accel_scale
                
                gyro_match = re.search(r'@g\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', raw_string)
                if gyro_match:
                    gx = float(gyro_match.group(1)) * self.gyro_scale
                    gy = float(gyro_match.group(2)) * self.gyro_scale
                    gz = float(gyro_match.group(3)) * self.gyro_scale
                    has_physics_data = True

            else:
                numbers = re.findall(r'[-+]?\d*\.?\d+', raw_string)
                if len(numbers) >= 3:
                    roll = float(numbers[0])
                    pitch = float(numbers[1])
                    yaw = float(numbers[2])
                    if len(numbers) >= 4:
                        heave = float(numbers[3])
                    self.mru_data_updated.emit(roll, pitch, yaw, heave)
                return 

            if has_physics_data:
                self._process_imu_physics(ax, ay, az, gx, gy, gz)

        except ValueError:
            return  

    def _parse_witmotion_packet(self, packet):
        """Decodes the 11 validated bytes from WitMotion."""
        self.debug_signal.emit( f"Tipo pacote = 0x{packet[1]:02X}")
        packet_type = packet[1]
        
        try:
            if packet_type == 0x51: # Accelerometer
                ax, ay, az = struct.unpack('<hhh', packet[2:8])
                self._wit_accel = [
                    (ax / 32768.0) * 16 * 9.80665 * self.accel_scale,
                    (ay / 32768.0) * 16 * 9.80665 * self.accel_scale,
                    (az / 32768.0) * 16 * 9.80665 * self.accel_scale  ]

            elif packet_type == 0x52: # Gyroscope
                gx, gy, gz = struct.unpack('<hhh', packet[2:8])
                self._wit_gyro = [
                    (gx / 32768.0) * 2000 * (3.14159265 / 180.0) * self.gyro_scale,
                    (gy / 32768.0) * 2000 * (3.14159265 / 180.0) * self.gyro_scale,
                    (gz / 32768.0) * 2000 * (3.14159265 / 180.0) * self.gyro_scale  ]

            elif packet_type == 0x53: 
                r, p, y = struct.unpack('<hhh', packet[2:8])
                self._wit_angle = [
                    (r / 32768.0) * 180.0,
                    (p / 32768.0) * 180.0,
                    (y / 32768.0) * 180.0   ]
                
                self.debug_signal.emit(
                    f"Roll={self._wit_angle[0]:.2f} "
                    f"Pitch={self._wit_angle[1]:.2f} "
                    f"Yaw={self._wit_angle[2]:.2f}"
                )
                self.mru_data_updated.emit(
                    self._wit_angle[0],  # Roll
                    self._wit_angle[1],  # Pitch
                    self._wit_angle[2],  # Yaw
                    0.0                  # Heave
                )
                
                self._process_imu_physics(
                    self._wit_accel[0], self._wit_accel[1], self._wit_accel[2],
                    self._wit_gyro[0], self._wit_gyro[1], self._wit_gyro[2],
                    self._wit_angle[0], self._wit_angle[1], self._wit_angle[2] )
                
        except Exception as e:
            self.debug_signal.emit(f"[Error] Sensor WitMotion: {e}")
    
    def _check_witmotion_checksum(self, packet: bytearray) -> bool:
        """Validates WitMotion binary packets (11 bytes)."""
        if len(packet) < 11:
            return False
        return (sum(packet[:-1]) & 0xFF) == packet[10]

    def _process_imu_physics(self, ax, ay, az, gx, gy, gz, roll, pitch, yaw):
        """Calculates delta_t and emits the IMUData object containing the observations for the EKF."""
        current_time = time.time()
        
        dt = 0.01  # 100Hz
        if self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            dt = max(0.001, min(dt, 0.1))  # Limits to between 1ms and 100ms
        self.last_timestamp = current_time
        
        imu_pkg = IMUData(
            accel_x=ax, accel_y=ay, accel_z=az,
            gyro_x=gx, gyro_y=gy, gyro_z=gz,
            roll=roll, pitch=pitch, yaw=yaw,  
            delta_t=dt, timestamp=current_time )
        
        self.imu_data_for_fusion.emit(imu_pkg) 

    def calibrate_gyro_bias(self, duration_seconds=5):
        """Calibrates the gyroscope bias. Uses QTimer to avoid freezing the QGIS interface."""
        if getattr(self, '_is_calibrating_gyro', False):
            return
            
        self._is_calibrating_gyro = True
        self._calib_gyro_sum = 0.0
        self._calib_gyro_count = 0

        def collect_bias(imu_pkg):
            self._calib_gyro_sum += imu_pkg.gyro_z
            self._calib_gyro_count += 1
        
        self.imu_data_for_fusion.connect(collect_bias)
        
        def finish_calibration():
            try:
                self.imu_data_for_fusion.disconnect(collect_bias)
            except TypeError:
                pass
                
            self._is_calibrating_gyro = False
            
            # Calculates the final bias and outputs the initial condition for the EKF
            if self._calib_gyro_count > 0:
                self.gyro_bias_z = self._calib_gyro_sum / self._calib_gyro_count
                self.debug_signal.emit(f"Calibrated gyroscope bias: {self.gyro_bias_z:.6f} rad/s")
                
                self.gyro_bias_estimated.emit(self.gyro_bias_z)
            else:
                self.debug_signal.emit("Calibration failure.")

        QTimer.singleShot(duration_seconds * 1000, finish_calibration)
    
    def calibrate_accel_bias(self, duration_seconds=5):
        """Calibrates the accelerometer bias."""
        if getattr(self, '_is_calibrating_accel', False):
            self.debug_signal.emit("Accelerometer calibration already in progress.")
            return

        self._is_calibrating_accel = True
        self.debug_signal.emit(f"Calibrating accelerometer for {duration_seconds}s...")
        self._calib_ax_sum = 0.0
        self._calib_ay_sum = 0.0
        self._calib_accel_count = 0
        
        def collect_bias(imu_pkg):
            self._calib_ax_sum += imu_pkg.accel_x
            self._calib_ay_sum += imu_pkg.accel_y
            self._calib_accel_count += 1

        self.imu_data_for_fusion.connect(collect_bias)
        
        def finish_calibration():
            try:
                # Disconnect to stop accumulating data
                self.imu_data_for_fusion.disconnect(collect_bias)
            except TypeError:
                pass
                
            self._is_calibrating_accel = False

            if self._calib_accel_count > 0:
                self.accel_bias_x = self._calib_ax_sum / self._calib_accel_count
                self.accel_bias_y = self._calib_ay_sum / self._calib_accel_count
                self.debug_signal.emit(f"Calibrated accelerometer bias: ax={self.accel_bias_x:.6f}, ay={self.accel_bias_y:.6f} m/s²")
                
                self.accel_bias_estimated.emit(self.accel_bias_x, self.accel_bias_y)
            else:
                self.debug_signal.emit("Accelerometer calibration failure")

        QTimer.singleShot(duration_seconds * 1000, finish_calibration)

    def set_sensor_scales(self, accel_scale=1.0, gyro_scale=1.0):
        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        self.debug_signal.emit(f"Escalas: accel={accel_scale}, gyro={gyro_scale}")