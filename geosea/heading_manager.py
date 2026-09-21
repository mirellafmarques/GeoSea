from qgis.PyQt.QtCore import QObject, pyqtSignal, QIODevice
from qgis.PyQt.QtSerialPort import QSerialPort
import time, re, math, traceback
from .filter_kalman import HeadingData

class HeadingManager(QObject):
    # Communication signals
    connection_status = pyqtSignal(bool, str) 
    raw_data_received = pyqtSignal(str)

    heading_updated = pyqtSignal(float, float, float, float)  # true, mag, rot, desvio
    heading_for_fusion = pyqtSignal(object)  # FusionEngine
    
    true_heading_updated = pyqtSignal(float)
    magnetic_heading_updated = pyqtSignal(float)
    rot_updated = pyqtSignal(float)
    deviation_updated = pyqtSignal(float)


    def __init__(self):
        super().__init__()
        self.serial = None

        self.last_true_heading = 0.0
        self.last_magnetic_heading = 0.0
        self.last_rot = 0.0      # ROT (degrees/minute)
        self.last_deviation = 0.0
        self.last_reliability = 0.95
        self.last_timestamp = time.time()
       
        self.rot_timeout = 5.0  # ROT timeout (seconds)
        

    def connect_serial(self, port_name, baud_rate, data_bits):
        self.disconnect_heading()
        self.serial = QSerialPort()
        self.serial.setPortName(port_name)
        self.serial.setBaudRate(int(baud_rate))
        self.serial.setDataBits(data_bits)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setParity(QSerialPort.NoParity)

        if self.serial.open(QIODevice.ReadOnly):
            self.serial.readyRead.connect(self.read_serial_data)
        else:
            erro = self.serial.errorString()
            self.connection_status.emit(False, f"Falha: {erro}")

    def disconnect_heading(self):
        if self.serial and self.serial.isOpen():
            self.serial.close()
        self.serial = None

    def read_serial_data(self):
        """Reads the data and processes the NMEA sentences."""
        while self.serial and self.serial.canReadLine():
            try:
                line = self.serial.readLine().data().decode('ascii', errors='ignore').strip()
                if line:
                    self.raw_data_received.emit(line)                    
                    self.parse_nmea_sentence(line)                    
            except Exception as e:
                self.raw_data_received.emit(f"ERROR: {str(e)}")


    def parse_nmea_sentence(self, sentence):
        """Accepted sentences:
         $xxHDT: Heading Verdadeiro (True)
         $xxHDG: Heading Magnético + Desvio + Variação
         $xxHDM: Heading Magnético
         $xxROT: Rate Of Turn (Taxa de Giro)
         $xxTHS: True Heading and Status """
        try:
            if '*' in sentence:
                sentence = sentence.split('*')[0]
            
            parts = sentence.split(',')
            if len(parts) < 2:
                return
            
            sentence_type = parts[0][3:]  
            
            # HDT - True Heading
            if sentence_type == 'HDT':
                if len(parts) >= 3 and parts[2] == 'T':
                    true_heading = float(parts[1])
                    self.last_true_heading = true_heading
                    self.true_heading_updated.emit(true_heading)
                    self.raw_data_received.emit(f" Heading True: {true_heading:.2f}°")
                    self._send_to_fusion()

            # HDG: Magnetic Heading + Deviation + Variation
            elif sentence_type == 'HDG':
                if len(parts) >= 5:
                    # Magnetic
                    mag_heading = float(parts[1])
                    self.last_magnetic_heading = mag_heading
                    self.magnetic_heading_updated.emit(mag_heading)
                    
                    # Deviation
                    if parts[3]:
                        deviation = float(parts[3])
                        # E = positive, W = negative
                        if len(parts) > 4 and parts[4] == 'W':
                            deviation = -deviation
                        self.last_deviation = deviation
                        self.deviation_updated.emit(deviation)
                    
                    # Calculates True heading from magnetic + deviation + variation
                    if len(parts) > 5 and parts[5]:
                        variation = float(parts[5])
                        if len(parts) > 6 and parts[6] == 'W':
                            variation = -variation
                        true_heading = mag_heading + deviation + variation
                        self.last_true_heading = true_heading % 360
                        self.true_heading_updated.emit(self.last_true_heading)
                    self.raw_data_received.emit(f"Heading Mag: {mag_heading:.2f}°, Deviation: {self.last_deviation:.2f}°")
                    self._send_to_fusion()
            
            # HDM: Magnetic
            elif sentence_type == 'HDM':
                if len(parts) >= 3 and parts[2] == 'M':
                    mag_heading = float(parts[1])
                    self.last_magnetic_heading = mag_heading
                    self.magnetic_heading_updated.emit(mag_heading)
                    self.raw_data_received.emit(f"Heading Mag: {mag_heading:.2f}°")
                    self._send_to_fusion()  

             # ROT: Rate Of Turn (positive = bombordo, negativo = bombordo). Unit: degrees per minute
            elif sentence_type == 'ROT':
                if len(parts) >= 2 and parts[1]:
                    rot_raw = float(parts[1])
                    # Converts to radians per second (1 degree/minute = 0.000290888 rad/s)
                    self.last_rot = rot_raw * 0.000290888  # rad/s
                    
                    # Checks status (A = valid, V = invalid)
                    if len(parts) >= 3 and parts[2] == 'A':
                        self.rot_updated.emit(rot_raw)  # degrees/minute
                        self.raw_data_received.emit(f"ROT: {rot_raw:.1f}°/min ({self.last_rot:.4f} rad/s)")
                    else:
                        self.raw_data_received.emit(f"ROT inválido: {rot_raw}")
                    
                    self._send_to_fusion()
            
            # THS: True Heading and Status
            elif sentence_type == 'THS':
                if len(parts) >= 3 and parts[2] == 'T':
                    true_heading = float(parts[1])
                    self.last_true_heading = true_heading 
                    self.true_heading_updated.emit(true_heading)
                    
                    # If there is ROT on the THS
                    if len(parts) >= 4 and parts[3]:
                        rot_raw = float(parts[3])
                        self.last_rot = rot_raw * 0.000290888
                        self.rot_updated.emit(rot_raw)
                    
                    self.raw_data_received.emit(f" THS True: {true_heading:.2f}°, ROT: {rot_raw if 'rot_raw' in locals() else 0:.1f}°/min")
                    self._send_to_fusion()
            
            # Proprietary protocols such as Garmin heading ($PGRMZ)
            elif sentence_type.startswith('PGRM'):
                self.raw_data_received.emit(f"Proprietary protocol: {sentence_type}")
            
        except Exception as e:
            self.raw_data_received.emit(f" Error: {str(e)}")

    def _send_to_fusion(self):
        """Sends the data to the FusionEngine."""
        try:
            # Calculates reliability based on available data
            reliability = self._calculate_reliability()

            # Use the magnetic one if 'true' is not available.
            angle_to_send = self.last_true_heading
            if angle_to_send == 0.0 and self.last_magnetic_heading != 0.0:
                angle_to_send = self.last_magnetic_heading
                reliability *= 0.8  # Penalizes reliability due to it being the magnetic north
            
            if angle_to_send > 0:
            
                # Creates the Heading package for the FKE
                heading_pkg = HeadingData( 
                    angle=angle_to_send,
                    reliability=reliability,
                    timestamp=time.time()  )
                
                self.heading_for_fusion.emit(heading_pkg)
                
                self.heading_updated.emit(
                    self.last_true_heading,
                    self.last_magnetic_heading,
                    self.last_rot * 3437.75,  # Converts rad/s to degrees/minute for display
                    self.last_deviation  )

            else:
                print(f"[HEADING] Invalid angle ({angle_to_send}).")

        except Exception as e:
            traceback.print_exc()
            self.raw_data_received.emit(f" Error sending data: {e}")

    def _calculate_reliability(self):
        """Calculates reliability based on the time since the last update."""
        time_since_update = time.time() - self.last_timestamp
        
        # Attributes reliability to more recent data.
        if time_since_update < 0.1:
            reliability = 0.99
        elif time_since_update < 0.5:
            reliability = 0.95
        elif time_since_update < 1.0:
            reliability = 0.90
        elif time_since_update < 2.0:
            reliability = 0.80
        else:
            reliability = 0.70
        self.last_timestamp = time.time()
        return reliability

    def get_current_heading(self):
        """Returns the current heading."""
        return {'true': self.last_true_heading,
            'magnetic': self.last_magnetic_heading,
            'rot': self.last_rot,
            'deviation': self.last_deviation,
            'reliability': self._calculate_reliability() }

    def send_heading_manually(self, true_heading=None, magnetic_heading=None, rot=None, deviation=None):
        """Allows data to be sent manually for use in tests."""
        if true_heading is not None:
            self.last_true_heading = true_heading % 360
        
        if magnetic_heading is not None:
            self.last_magnetic_heading = magnetic_heading % 360
        if rot is not None:
            self.last_rot = rot * 0.000290888  # rad/s
        if deviation is not None:
            self.last_deviation = deviation
        
        self._send_to_fusion()
        self.raw_data_received.emit(f"Heading: True={self.last_true_heading:.1f}°")