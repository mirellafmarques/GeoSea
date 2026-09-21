from qgis.PyQt.QtCore import QObject, pyqtSignal, QIODevice
from qgis.PyQt.QtSerialPort import QSerialPort

class MeteoManager(QObject):
    raw_data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool, str)
    meteo_data_parsed = pyqtSignal(dict) 

    def __init__(self):
        super().__init__()
        self.serial = QSerialPort()
        self.serial.readyRead.connect(self.read_data)
        self.buffer = bytearray()
        
        # Store the latest observations
        self.current_weather = {'speed': '--', 'dir': '--', 'temp_ar': '--',
            'hum': '--', 'baro': '--', 'temp_agua': '--' }

    def connect_serial(self, port_name, baudrate_str, data_bits_str):
        if self.serial.isOpen():
            self.serial.close()

        self.serial.setPortName(port_name)
        
        baud = int(baudrate_str) if baudrate_str.isdigit() else 4800
        self.serial.setBaudRate(baud)
        if "7" in data_bits_str:
            self.serial.setDataBits(QSerialPort.Data7)
        else:
            self.serial.setDataBits(QSerialPort.Data8)

        # Parity
        self.serial.setParity(QSerialPort.NoParity)

        # Stop bits
        self.serial.setStopBits(QSerialPort.OneStop)

        if self.serial.open(QIODevice.ReadOnly):
            self.connection_status.emit(True, f"Weather station connected to {port_name}")
            return True
        else:
            erro = self.serial.errorString()
            self.connection_status.emit(False, f"Connection error: {erro}")
            return False

    def disconnect_meteo(self, silencioso=False):
        if self.serial and self.serial.isOpen():
            self.serial.close()
        self.serial = None 
        if not silencioso:
            self.connection_status.emit(False, "Disconnected")

    def read_data(self):
        """Reads the raw data."""
        if self.serial.bytesAvailable():
            data = self.serial.readAll()
            self.buffer.extend(data.data())
            
            while b'\n' in self.buffer:
                linha, self.buffer = self.buffer.split(b'\n', 1)
                texto_limpo = linha.decode('ascii', errors='ignore').strip()
                
                if texto_limpo:
                    self.raw_data_received.emit(texto_limpo)
                    
                    if texto_limpo.startswith('$') or texto_limpo.startswith('!'):
                        self.parse_nmea_meteo(texto_limpo)

    def parse_nmea_meteo(self, sentence):
        """Processes NMEA sentences."""
        parts = sentence.split('*')[0].split(',')
        if len(parts) < 2: return
        
        # Uses the last 3 fields
        header = parts[0][-3:] 
        updated = False
        
        try:
            # MWV - Wind Speed and Angle
            if header == 'MWV' and len(parts) >= 6:
                if parts[1]: self.current_weather['dir'] = f"{float(parts[1]):.0f}°"
                if parts[3]: self.current_weather['speed'] = f"{float(parts[3]):.1f} kt"
                updated = True
                
            # MDA - Meteorological Composite
            elif header == 'MDA':
                if len(parts) > 3 and parts[3]: 
                    self.current_weather['baro'] = f"{float(parts[3]) * 1000:.1f} hPa" # Converte Bares para hPa
                if len(parts) > 5 and parts[5]: 
                    self.current_weather['temp_ar'] = f"{float(parts[5]):.1f} °C"
                if len(parts) > 9 and parts[9]: 
                    self.current_weather['hum'] = f"{float(parts[9]):.0f} %"
                updated = True
                
            # MTW - Water Temperature
            elif header == 'MTW' and len(parts) >= 3:
                if parts[1]: self.current_weather['temp_agua'] = f"{float(parts[1]):.1f} °C"
                updated = True
                
            if updated:
                self.meteo_data_parsed.emit(self.current_weather)
                
        except Exception:
            pass 