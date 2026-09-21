from qgis.core import (QgsGpsDetector, QgsGpsConnection, QgsApplication, QgsGpsInformation,
    QgsPointXY, QgsGeometry, QgsFeature, QgsVectorLayer,
    QgsField, QgsProject, QgsMarkerSymbol, QgsRendererCategory, 
    QgsCategorizedSymbolRenderer)
from PyQt5.QtCore import (Qt, QPoint, QTimer, QVariant, QThread, pyqtSignal, QObject)
from PyQt5.QtWidgets import (QMainWindow, QFileDialog, QMessageBox, QDockWidget, QWidget,
    QVBoxLayout, QLabel, QHBoxLayout, QPushButton)
from PyQt5.QtGui import QColor, QFont, QPalette, QPainter
import serial, serial.tools.list_ports, time, traceback, math, socket
from datetime import datetime, timedelta
from pyproj import CRS, Transformer
from .filter_kalman import FusionEngine, GNSSData, FusionState, IMUData, HeadingData


# CLASSE NMEAWorker
class NMEAWorker(QObject):
    """Classe dedicada para conexões de rede (UDP/TCP)"""   
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    positionReceived = pyqtSignal(float, float, float, float, float)  # lat, lon, alt, rumo, velocidade

    def __init__(self, ip=None, port=None, protocol="UDP"):
        super().__init__()
        self.ip = ip
        self.port = port
        self.protocol = protocol
        self.running = False
        self.socket = None

    def run(self):
        """Método principal do worker (executado em thread separada)"""
        self.running = True
        self.connection_status.emit(True)        
        try:
            if self.protocol == "UDP":
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.settimeout(1.0)
                self.data_received.emit(f"[INFO] Escutando UDP em {self.ip}:{self.port}")
                
                while self.running:
                    try:
                        data, addr = self.socket.recvfrom(1024)
                        line = data.decode('ascii', errors='ignore').strip()
                        self.data_received.emit(line)
                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.error_occurred.emit(f"Erro na leitura UDP: {e}")
                        time.sleep(0.1)
                        
            elif self.protocol == "TCP":
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.ip, self.port))
                self.socket.settimeout(1.0)
                self.data_received.emit(f"[INFO] Conectado TCP em {self.ip}:{self.port}")
                
                while self.running:
                    try:
                        data = self.socket.recv(1024)
                        if data:
                            line = data.decode('ascii', errors='ignore').strip()
                            self.data_received.emit(line)
                        else:
                            break
                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.error_occurred.emit(f"Erro na leitura: {e}")
                        break
                        
        except Exception as e:
            self.error_occurred.emit(f"Erro ao iniciar conexão {self.protocol}: {e}")
            self.running = False
        finally:
            self.stop()

    def stop(self):
        """Para o worker e fecha conexões"""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connection_status.emit(False)


# CLASSE GNSSManager (GERENCIADOR PRINCIPAL)
class GNSSManager(QObject):
    """Gerencia todas as conexões GNSS (Serial, UDP, TCP)""" 
       
    gps_data_received = pyqtSignal(dict)  # Dados GNSS formatados
    connection_status = pyqtSignal(str)   # Status da conexão
    raw_data_received = pyqtSignal(str)   # Dados NMEA bruto
    gnss_data_for_fusion = pyqtSignal(object)  # Emite GNSSData

    vdop_updated = pyqtSignal(float)

    fusion_state_updated = pyqtSignal(object)  # Estado fusionado
    kalman_debug = pyqtSignal(str)             # Mensagens de debug
        
    def __init__(self):
        super().__init__()
        self.gps_connection = None        # Para conexão QGIS GPS
        self.nmea_worker = None           # Para conexões NMEA de rede
        self.network_thread = None        # Thread para worker de rede
        self.serial_connection = None     # Para conexão serial direta
        self.current_port = None
        self.is_connected = False
        self.timer = QTimer()             # Timer para leitura periódica
        self.timer.timeout.connect(self._check_connections)

        # Fusão Kalman
        self.fusion_engine = FusionEngine()
        self.fusion_engine.state_updated.connect(self._on_fusion_state_updated)
        self.fusion_engine.debug_signal.connect(self._on_fusion_debug)
        self.gnss_data_for_fusion.connect(self.fusion_engine.process_gnss)

        # Variáveis para conversão UTM
        self.current_utm_zone = None
        self.current_utm_crs = None
        self.is_south_hemisphere = True
        
        # Cache de transformers para diferentes fusos
        self.utm_transformers = {}      # (zone, is_south) -> transformer
        self.latlon_transformers = {}   # (zone, is_south) -> transformer
        
        self.last_latitude = None
        self.last_longitude = None
        self.last_speed = 0.0
        self.last_cog = 0.0
                
        self.primeiro_zoom_feito = False
        self.last_vdop = None

    def _get_utm_transformer(self, zone, is_south):
        """Obtém ou cria um transformer UTM para o fuso especificado."""
        key = (zone, is_south)
        if key not in self.utm_transformers:
            crs_utm = CRS.from_dict({
                "proj": "utm",
                "zone": zone,
                "south": is_south})
            self.utm_transformers[key] = Transformer.from_crs(
                "EPSG:4326", crs_utm, always_xy=True)
        return self.utm_transformers[key]
    

    def _get_latlon_transformer(self, zone, is_south):
        """Obtém ou cria um transformer de UTM para Lat/Long."""
        key = (zone, is_south)
        if key not in self.latlon_transformers:
            crs_utm = CRS.from_dict({
                "proj": "utm",
                "zone": zone,
                "south": is_south})
            self.latlon_transformers[key] = Transformer.from_crs(
                crs_utm, "EPSG:4326", always_xy=True)
        return self.latlon_transformers[key]
    

    def _latlon_to_utm(self, lat, lon):
        """Converte latitude/longitude para coordenadas UTM. Detecta automaticamente o fuso correto."""
        if lat is None or lon is None:
            return None, None, None, None
        zone = int((lon + 180) / 6) + 1 # Calcula o fuso baseado na longitude
        is_south = lat < 0
        
        if self.current_utm_zone != zone: # Se o fuso mudou, atualiza e emite mensagem
            old_zone = self.current_utm_zone
            self.current_utm_zone = zone
            self.current_utm_crs = f"UTM {zone}{'S' if is_south else 'N'}"
            self.is_south_hemisphere = is_south
            self.kalman_debug.emit(f"[UTM] Fuso alterado: {old_zone} -> {zone} (CRS: {self.current_utm_crs})")
        transformer = self._get_utm_transformer(zone, is_south)
        easting, northing = transformer.transform(lon, lat)
        return easting, northing, zone, is_south
    

    def utm_to_latlon(self, easting, northing, zone=None, is_south=None):
        """ Converte coordenadas UTM para Lat/Lon. Usa o fuso atual se não for especificado."""
        if zone is None:
            zone = self.current_utm_zone
        if is_south is None:
            is_south = self.is_south_hemisphere
        if zone is None: # Se não temos fuso definido ainda, retorna (0,0)
            self.kalman_debug.emit("[AVISO] utm_to_latlon chamado sem fuso definido")
            return 0.0, 0.0
        transformer = self._get_latlon_transformer(zone, is_south)
        lon, lat = transformer.transform(easting, northing)
        return lat, lon
    

    # Fusão pelo FKE
    def _process_gnss_for_fusion(self, lat, lon, speed, cog, hdop, altitude=0.0):
        """Processa dados GNSS e injeta no filtro de Kalman estendido."""
        try:
            if lat is None or lon is None:
                return False
            easting, northing, zone, is_south = self._latlon_to_utm(lat, lon) # Converte para UTM
            if easting is None:
                return False
            
            vdop = self.last_vdop if self.last_vdop else 1.0
            accuracy = (hdop + vdop) * 1.5
            # Cria pacote GNSS
            gnss_pkg = GNSSData(
                easting=easting,
                northing=northing,
                altitude=0.0,
                speed=float(speed) if speed else 0.0,
                accuracy=float(accuracy),
                timestamp=time.time())            

            # Verifica se cog é válido
            if cog is not None and cog > 0:
                gnss_pkg.cog = cog
            else:
                gnss_pkg.cog = 0.0
            
            gnss_pkg.cog = cog if cog is not None else 0.0
            self.gnss_data_for_fusion.emit(gnss_pkg)

            return True
        except Exception as e:
            traceback.print_exc()
            return False
    

    def _process_cog_for_fusion(self, cog, reliability=0.95):
        """Processa dados do COG para o Kalman"""
        try:
            if cog is None:
                return
            cog_pkg = HeadingData(
                angle=float(cog),
                reliability=float(reliability),
                timestamp=time.time())
            self.fusion_engine.process_heading(cog_pkg)
        except Exception as e:
            self.kalman_debug.emit(f" Erro: {e}")
    
    def _process_gnss_speed_for_fusion(self, speed, cog):
        """Processa velocidade do GNSS para correção"""
        try:
            if speed is None or speed <= 0:
                return
            # Usa a velocidade do GNSS para corrigir vx, vy
            self.fusion_engine.process_gnss_speed(float(speed), float(cog) if cog else None)
        except Exception as e:
            self.kalman_debug.emit(f"Erro process_gnss_speed_for_fusion: {e}")

    def _on_fusion_state_updated(self, state: FusionState):
        try:            
            self.fusion_state_updated.emit(state)
        except Exception as e:
            traceback.print_exc()
    
    def _on_fusion_debug(self, msg):
        """Recebe mensagens de debug do filtro."""
        self.kalman_debug.emit(f"[KALMAN] {msg}")
    
    def get_fusion_state(self):
        """Retorna o estado atual do filtro."""
        return self.fusion_engine.get_current_state()
    
    def reset_fusion_filter(self):
        self.fusion_engine.reset_filter()
        self.kalman_debug.emit("Filtro de Kalman resetado")
    
    def _on_gps_data(self, gps_info):
        """Recebe dados GNSS do QGIS."""
        if gps_info.isValid():
                       
            # Prepara dados para a UI (dados brutos)
            data = {
                'latitude': gps_info.latitude,
                'longitude': gps_info.longitude,
                'altitude': gps_info.elevation if gps_info.elevation else 0,
                'speed': gps_info.speed if gps_info.speed else 0,
                'direction': gps_info.direction if gps_info.direction else 0,
                'hdop': gps_info.hdop if gps_info.hdop else 99,
                'satellites': gps_info.satellitesUsed if gps_info.satellitesUsed else 0,
                'quality': gps_info.quality,
                'timestamp': gps_info.utcDateTime.toString("hh:mm:ss") if gps_info.utcDateTime else "",
                'fix': self._get_fix_status(gps_info)
                }
            
             # Extrai dados
            lat = gps_info.latitude
            lon = gps_info.longitude
            speed = gps_info.speed if gps_info.speed else 0
            cog = gps_info.direction
            hdop = gps_info.hdop if gps_info.hdop else 99
           
            # Armazena últimos valores
            self.last_latitude = lat
            self.last_longitude = lon
            self.last_speed = speed
            self.last_cog = cog

            # Emite dados
            self.gps_data_received.emit(data)
            
            self._process_gnss_for_fusion(lat, lon, speed, cog, hdop, data['altitude'])
            
            vdop = hdop * 1.5  # estimativa
            self.vdop_updated.emit(vdop)

            
    def _on_nmea_data(self, nmea_string):
        """Processa dados do NMEA."""
        self.raw_data_received.emit(nmea_string)
        
        # Parse de sentença GGA para posição
        if nmea_string.startswith('$GNGGA') or nmea_string.startswith('$GPGGA'):
            self._parse_gga_sentence(nmea_string)
        
        # Parse de sentença RMC para velocidade e rumo
        elif nmea_string.startswith('$GNRMC') or nmea_string.startswith('$GPRMC'):
            self._parse_rmc_sentence(nmea_string)
        
        # Parse de sentença VTG para velocidade e rumo
        elif nmea_string.startswith('$GNVTG') or nmea_string.startswith('$GPVTG'):
            self._parse_vtg_sentence(nmea_string)
    
    def _parse_gga_sentence(self, nmea_string):
        """Parseia sentença GGA para posição"""
        try:
            parts = nmea_string.split(',')
            if len(parts) < 10:
                return
            
            # Extrai latitude
            lat_raw = parts[2]
            lat_dir = parts[3]
            if lat_raw and lat_dir:
                lat = self._convert_nmea_lat(lat_raw, lat_dir)
            else:
                return
            # Extrai longitude
            lon_raw = parts[4]
            lon_dir = parts[5]
            if lon_raw and lon_dir:
                lon = self._convert_nmea_lon(lon_raw, lon_dir)
            else:
                return
            
            # Extrai HDOP
            hdop = float(parts[8]) if parts[8] else 99.0
            # Injeta no Kalman
            self._process_gnss_for_fusion(lat, lon, self.last_speed, self.last_cog, hdop, )
        except Exception as e:
            self.kalman_debug.emit(f"Erro parse GGA: {e}")
    
    def _parse_rmc_sentence(self, nmea_string):
        """Parseia sentença RMC para velocidade e rumo (SOG e COG)."""
        try:
            parts = nmea_string.split(',')
            if len(parts) < 10:
                return
            
            # Extrai velocidade (nós)
            if parts[7]:
                self.last_speed = float(parts[7])
            
            # Extrai rumo (graus)
            if parts[8]:
                self.last_cog = float(parts[8])
                self._process_cog_for_fusion(self.last_cog)
            
        except Exception as e:
            self.kalman_debug.emit(f"Erro parse RMC: {e}")
    
    def _parse_vtg_sentence(self, nmea_string):
        """Parseia sentença VTG para velocidade e rumo"""
        try:
            parts = nmea_string.split(',')
            if len(parts) < 8:
                return
            
            # Extrai o COG
            if parts[1]:
                self.last_cog = float(parts[1])
                self._process_cog_for_fusion(self.last_cog)
            
            # Extrai velocidade (nós)
            if parts[5]:
                self.last_speed = float(parts[5])
            
        except Exception as e:
            self.kalman_debug.emit(f"Erro parse VTG: {e}")
    
    def _parse_gsa_sentence(self, nmea_string):
        try:
            parts = nmea_string.split(',')

            if len(parts) < 17:
                return

            pdop = float(parts[-3]) if parts[-3] else None
            hdop = float(parts[-2]) if parts[-2] else None
            vdop = float(parts[-1].split('*')[0]) if parts[-1] else None

            self.last_vdop = vdop
            self.kalman_debug.emit(f"[DOP] PDOP={pdop} HDOP={hdop} VDOP={vdop}")

        except Exception as e:
            self.kalman_debug.emit(f"Erro parse GSA: {e}")
    
    def _convert_nmea_lat(self, raw_val, direction):
        """Converte latitude NMEA para decimal."""
        if not raw_val:
            return 0.0
        try:
            d = float(raw_val[:2])
            m = float(raw_val[2:]) / 60
            return (d + m) * (-1 if direction == 'S' else 1)
        except:
            return 0.0
    
    def _convert_nmea_lon(self, raw_val, direction):
        """Converte longitude NMEA para decimal"""
        if not raw_val:
            return 0.0
        try:
            d = float(raw_val[:3])
            m = float(raw_val[3:]) / 60
            return (d + m) * (-1 if direction == 'W' else 1)
        except:
            return 0.0


    # Conexão GNSS com a Ferramenta GPS nativa do QGIS
    def connect_gnss(self, port):
        """Conecta ao dispositivo GNSS usando API do QGIS"""
        self.current_port = port
        
        # Verifica conexão existente
        registry = QgsApplication.gpsConnectionRegistry()
        connections = registry.connectionList()
        
        if connections:
            self.gps_connection = connections[0]
            self._setup_gps_connection()
            self.connection_status.emit("Conectado")
        else:
            # Cria nova conexão
            self.connection_status.emit(f"Conectando em {port}...")
            self.gps_detector = QgsGpsDetector(port)
            self.gps_detector.detected.connect(self._on_gps_detected)
            self.gps_detector.detectionFailed.connect(self._on_gps_failed)
            self.gps_detector.advance()

    def _on_gps_detected(self, connection):
        """GNSS detectado com sucesso"""
        self.gps_connection = connection
        self._setup_gps_connection()
        self.connection_status.emit("GNSS conectado!")

    def _on_gps_failed(self):
        """Falha na conexão"""
        self.connection_status.emit("Falha na conexão")
        self.disconnect_gnss()

    def _setup_gps_connection(self):
        """Configura a conexão GNSS do QGIS."""
        if self.gps_connection:
            self.gps_connection.stateChanged.connect(self._on_gps_data)
            
            self.gps_connection.nmeaSentenceReceived.connect(self._on_qgis_nmea_received)
            # Registra no QGIS
            registry = QgsApplication.gpsConnectionRegistry()
            registry.registerConnection(self.gps_connection)
            
            self.is_connected = True
            self.timer.start(100)  # Verifica conexões a cada 100ms

    def _on_qgis_nmea_received(self, nmea_string):
        if nmea_string:
            self.raw_data_received.emit(nmea_string.strip())

    def _get_fix_status(self, gps_info):
        """Retorna status do fix em texto legível"""
        fix_status = {
            1: "2D",
            2: "3D" }
        return fix_status.get(gps_info.quality, "NO FIX")

    # Conexão com o receptor (Serial/Rede)
    def connect_nmea(self, protocol, port=None, ip=None, network_port=None, baudrate=None, bytesize=None):
        """Conecta a fontes NMEA via comunicação Serial ou Rede (UDP/TCP)."""
        try:
            if protocol == "Serial":
                if not port:
                    raise ValueError("Porta serial não especificada")
                
                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=baudrate,
                    bytesize=bytesize,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1  )
                self.connection_status.emit(f"Conectado Serial: {port} @ {baudrate}bps, {bytesize} bits")
                self.timer.timeout.connect(self._read_serial)
                
            elif protocol in ["UDP", "TCP"]:
                if not ip or not network_port:
                    raise ValueError("IP e porta não especificados")
                
                # Cria worker em thread separada
                self.network_thread = QThread()
                self.nmea_worker = NMEAWorker(
                    ip=ip, 
                    port=network_port, 
                    protocol=protocol
                )
                self.nmea_worker.moveToThread(self.network_thread)
                
                # Conecta sinais
                self.nmea_worker.data_received.connect(self._on_nmea_data)
                self.nmea_worker.error_occurred.connect(self._on_nmea_error)
                self.nmea_worker.connection_status.connect(self._on_nmea_status)
                
                # Inicia thread
                self.network_thread.started.connect(self.nmea_worker.run)
                self.network_thread.start()
                
                self.connection_status.emit(f"Conectando {protocol}...")
                
            self.timer.start(100)
            self.is_connected = True
            
        except Exception as e:
            self.connection_status.emit(f"Erro: {str(e)}")
            self.disconnect_gnss()

    def _read_serial(self):
        """Lê dados da porta serial de forma segura sem travar o QGIS."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                while self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        self._on_nmea_data(line)
            except Exception as e:
                self._on_nmea_error(f"Erro serial: {e}")

    def _on_nmea_data(self, nmea_string):
        """Processa dados NMEA."""
        self.raw_data_received.emit(nmea_string)
        
        if nmea_string.startswith('$GNGGA'):
            pass

        elif nmea_string.startswith('$GPGSA') or nmea_string.startswith('$GNGSA'):
            self._parse_gsa_sentence(nmea_string)


    def _on_nmea_error(self, error_msg):
        """Informa os erros de conexão."""
        self.connection_status.emit(f"Erro NMEA: {error_msg}")
        self.disconnect_gnss()

    def _on_nmea_status(self, status):
        """Atualiza status da conexão."""
        if not status:
            self.connection_status.emit("Conexão perdida")

    def _check_connections(self):
        """Verifica periodicamente o estado das conexões."""
        if self.serial_connection and not self.serial_connection.is_open:
           self.disconnect_gnss()

    def disconnect_gnss(self):
        """Desconecta todas as conexões GNSS."""
        self.timer.stop()
        
        # Desconecta GNSS do QGIS
        if self.gps_connection:
            try:
                self.gps_connection.stateChanged.disconnect()
                registry = QgsApplication.gpsConnectionRegistry()
                registry.unregisterConnection(self.gps_connection)
                self.gps_connection.nmeaSentenceReceived.disconnect()
                self.gps_connection.close()
            except:
                pass
            self.gps_connection = None
        
        # Desconecta serial
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.serial_connection = None
        
        # Desconecta rede
        if self.nmea_worker:
            self.nmea_worker.stop()
        
        if self.network_thread and self.network_thread.isRunning():
            self.network_thread.quit()
            self.network_thread.wait()
            self.network_thread = None
            self.nmea_worker = None
        
        self.is_connected = False
        self.connection_status.emit("Desconectado")

    def get_available_ports(self):
        """Retorna portas seriais disponíveis"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)
        return ports

    def is_connected(self):
        """Retorna status da conexão"""
        return self.is_connected
    
    def start_gnss_connection(self):

        protocol = self.comboBox_protocol.currentText()

        if protocol == "Serial":
            port = self.comboBox_serial.currentText()
            baud = int(self.comboBox_baundrate.currentData())
            bits = int(self.comboBox_bits.currentData())

            self.gnss_manager.connect_nmea(
                protocol="Serial",
                port=port,
                baudrate=baud,
                bytesize=bits)

        elif protocol == "UDP":
            ip = self.lineEdit_ip.text()
            udp_port = int(self.lineEdit_udp_port.text())

            self.gnss_manager.connect_nmea(
                protocol="UDP",
                ip=ip,
                network_port=udp_port )

        elif protocol == "TCP":
            ip = self.lineEdit_ip.text()
            tcp_port = int(self.lineEdit_tcp_port.text())

            self.gnss_manager.connect_nmea(
                protocol="TCP",
                ip=ip,
                network_port=tcp_port  )

    def toggle_connection(self, checked):
        if checked:
            self.radioButton_connect.setText("Desconectar")
            self.connect_device()
        else:
            self.radioButton_connect.setText("Conectar")
            self.disconnect_device()