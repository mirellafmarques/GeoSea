from qgis.PyQt.QtCore import QObject, pyqtSignal, QTimer, QVariant, Qt
from qgis.PyQt.QtGui import QColor, QFont
from qgis.PyQt.QtWidgets import QFileDialog
from qgis.core import (QgsProject, QgsVectorLayer, QgsRasterLayer, QgsFeature, 
                       QgsGeometry, QgsPointXY, QgsField, QgsWkbTypes,
                       QgsCoordinateTransform, QgsCoordinateReferenceSystem,
                       QgsDistanceArea, QgsUnitTypes, QgsPalLayerSettings, QgsVectorLayerSimpleLabeling, 
                        QgsTextFormat, QgsTextBufferSettings)

from qgis.gui import QgsMapToolEmitPoint, QgsVertexMarker
import math, os, string


class PlanningManager(QObject):
    """Planning Logic Manager."""
    status_message = pyqtSignal(str, int) 
    layers_updated = pyqtSignal()         
    route_calculated = pyqtSignal(str, str, str) 
    vertices_extracted = pyqtSignal(list) 
    
    def __init__(self, iface):
        super().__init__()
        self.iface = iface
        self.canvas = iface.mapCanvas()        
        # Variables
        self.start_point = None
        self.end_point = None
        self.map_tool = None
        self.ref_geometry = None
        self.vertices_list = []        
        # Timer
        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self._check_route_logic)
        
    
    def import_raster(self):
        file_path, _ = QFileDialog.getOpenFileName(None, "Open Raster", "", "TIF (*.tif *.tiff);;All Files (*)")
        if file_path:
            layer = QgsRasterLayer(file_path, os.path.basename(file_path))
            if layer.isValid():
                QgsProject.instance().addMapLayer(layer)
                self.status_message.emit("Imported raster", 0)


    def import_vector(self):
        filtro = (
            "All Supported Vectors (*.shp *.gpkg *.kml *.dxf);;"
            "Shapefile (*.shp);;"
            "GeoPackage (*.gpkg);;"
            "Google Earth KML (*.kml);;"
            "AutoCAD DXF (*.dxf);;"
            "All files (*.*)")
        
        file_path, _ = QFileDialog.getOpenFileName(None, "Open", "", filtro)
        
        if file_path:
            layer = QgsVectorLayer(file_path, os.path.basename(file_path), "ogr")
            if layer.isValid():
                QgsProject.instance().addMapLayer(layer)
                
                if hasattr(self, 'status_message'):
                    self.status_message.emit(f"Imported vector: {os.path.basename(file_path)}", 0)
            else:
                if hasattr(self, 'iface'):
                    self.iface.messageBar().pushMessage("GeoSea", "Error: The vector file is invalid or empty.", level=2)

    def activate_drawing_tool(self):
        self.map_tool = QgsMapToolEmitPoint(self.canvas)
        self.map_tool.canvasClicked.connect(self._on_canvas_clicked)
        self.canvas.setMapTool(self.map_tool)
        self.status_message.emit("Click on two points on the map", 1)

    def generate_parallel_lines(self, qtd, dist):
        if not self.ref_geometry:
            self.status_message.emit("Set the reference", 2)
            return
        try:
            n = int(qtd)
            d = float(dist.replace(',', '.')) # Accepts a comma or a period
        except:
            return

        # Reference Line
        layer_name = "GeoSea_Reference_Line"
        layers = QgsProject.instance().mapLayersByName(layer_name)        
        if not layers:
            self.status_message.emit("Layer not found. Create the line.", 2)
            return            
        layer = layers[0]
        pr = layer.dataProvider()

        current_fields = [field.name() for field in layer.fields()]
        
        fields_to_add = []
        if "lado" not in current_fields:
            fields_to_add.append(QgsField("lado", QVariant.String))
        if "offset" not in current_fields:
            fields_to_add.append(QgsField("offset", QVariant.Double))            
        if fields_to_add:
            pr.addAttributes(fields_to_add)
            layer.updateFields()

        # Generate the parallel lines
        features_para_adicionar = []
        for i in range(1, n + 1):
            for side in [1, -1]:
                off = d * i * side
                geom = self._create_parallel_geometry(self.ref_geometry, off)
                if geom:
                    f = QgsFeature(layer.fields())
                    f.setGeometry(geom)
                    
                    idx_lado = layer.fields().indexOf("lado")
                    idx_off = layer.fields().indexOf("offset")
                    if idx_lado >= 0:
                        f.setAttribute(idx_lado, "Pos" if side > 0 else "Neg")
                    if idx_off >= 0:
                        f.setAttribute(idx_off, off)
                        
                    features_para_adicionar.append(f)
                    
        # Saves the parallel lines along with the reference line to the temporary layer
        if features_para_adicionar:
            layer.startEditing()
            pr.addFeatures(features_para_adicionar)
            layer.commitChanges()
            layer.updateExtents()
            layer.triggerRepaint()
            self.status_message.emit(f"Success! {n*2} parallels added to the layer.", 0)
        else:
            self.status_message.emit("Failed to calculate the parallel lines.", 2)

    def extract_vertices_from_layer(self, layer_id):
        layer = QgsProject.instance().mapLayer(layer_id)
        if not layer: return

        self.vertices_list = []
        letras = list(string.ascii_uppercase)
        extracted_data = []

        for i, feat in enumerate(layer.getFeatures()):
            geom = feat.geometry()
            if geom.isMultipart():
                lines = geom.asMultiPolyline()
                if not lines: continue
                line = lines[0]
            else:
                line = geom.asPolyline()
            
            if not line: 
                continue

            p_ini, p_fim = line[0], line[-1]
            letra = letras[i % 26]
            
            dx = p_fim.x() - p_ini.x()
            dy = p_fim.y() - p_ini.y()
            azimute = (math.degrees(math.atan2(dx, dy)) + 360) % 360

            row_data = [
                f"{letra}1",          # Starting vertex
                f"Linha {letra}",     # Line name
                f"{p_ini.x():.2f}",
                f"{p_ini.y():.2f}",
                "---",
                "---",                # Azimuth V1
                f"{letra}2",          # Final vertex
                f"{p_fim.x():.2f}",
                f"{p_fim.y():.2f}",
                "---",
                "---",                # Azimuth V2
                "SENTIDO_12"          
            ]
            extracted_data.append(row_data)

            # Saves the coordinates
            self.vertices_list.append((f"{letra}1", p_ini))
            self.vertices_list.append((f"{letra}2", p_fim))
               
        if self.vertices_list:
            crs = layer.crs().authid()
            layer_name = "GeoSea_Vertex"
            camadas_existentes = QgsProject.instance().mapLayersByName(layer_name)
            
            if camadas_existentes:
                vl_points = camadas_existentes[0]
                pr = vl_points.dataProvider()
                ids_antigos = [f.id() for f in vl_points.getFeatures()]
                if ids_antigos:
                    pr.deleteFeatures(ids_antigos)
            else:
                vl_points = QgsVectorLayer(f"Point?crs={crs}", layer_name, "memory")
                pr = vl_points.dataProvider()
                pr.addAttributes([QgsField("id_vertice", QVariant.String)])
                vl_points.updateFields()
                
                # Label configuration
                label_settings = QgsPalLayerSettings()
                label_settings.fieldName = "id_vertice"
                label_settings.placement = QgsPalLayerSettings.OrderedPositionsAroundPoint

                # Font
                text_format = QgsTextFormat()
                font = QFont("Arial")
                font.setBold(True)
                text_format.setFont(font)
                text_format.setSize(9)
                text_format.setColor(QColor(255, 255, 255))  # Color: white

                # Text Buffer
                buffer_settings = QgsTextBufferSettings()
                buffer_settings.setEnabled(True)
                buffer_settings.setSize(1.0)
                buffer_settings.setColor(QColor(0, 0, 0, 180)) # preto transparente

                text_format.setBuffer(buffer_settings)
                label_settings.setFormat(text_format)
                labeling = QgsVectorLayerSimpleLabeling(label_settings)
                vl_points.setLabeling(labeling)
                vl_points.setLabelsEnabled(True)
                
                QgsProject.instance().addMapLayer(vl_points)

            # Creates new points
            features = []
            campos = vl_points.fields() 
            idx = campos.indexOf("id_vertice")
            
            for nome_id, pt in self.vertices_list:
                f = QgsFeature(campos) 
                f.setGeometry(QgsGeometry.fromPointXY(pt))
                if idx >= 0:
                    f.setAttribute(idx, nome_id)
                features.append(f)
                
            pr.addFeatures(features)
            vl_points.updateExtents()
            vl_points.triggerRepaint()

            symbol = vl_points.renderer().symbol()
            symbol.setSize(3.5)
            symbol.setColor(QColor(0, 180, 220))
            symbol.symbolLayer(0).setStrokeColor(QColor(255,255,255))
            symbol.symbolLayer(0).setStrokeWidth(0.5)

        self.vertices_extracted.emit(extracted_data)

    def invert_line_direction(self, layer_id, feature_id):
        """Reverses the geometry of the selected line. Works for simple and multipart lines."""
        layer = QgsProject.instance().mapLayer(layer_id)
        if not layer:
            return

        feat = layer.getFeature(feature_id)
        if not feat or not feat.isValid():
            return

        geom = feat.geometry()
        if geom is None or geom.isEmpty():
            return

        if geom.isMultipart():
            lines = geom.asMultiPolyline()
            if not lines:
                return

            reversed_lines = [list(reversed(line)) for line in lines]
            new_geom = QgsGeometry.fromMultiPolylineXY(reversed_lines)

        else:
            line = geom.asPolyline()
            if not line:
                return
            line.reverse()
            new_geom = QgsGeometry.fromPolylineXY(line)

        if not layer.isEditable():
            layer.startEditing()
        ok = layer.changeGeometry(feature_id, new_geom)
        if ok:
            layer.commitChanges()
        else:
            layer.rollBack()
            self.status_message.emit("Error inverting geometry!", 2)
            return

        # Updates the vertex table
        self.extract_vertices_from_layer(layer_id)
        self.current_layer_id = layer_id

        try:
            row = self.tableWidget.currentRow()
            if row >= 0:
                col_sentido = 11  
                item = self.tableWidget.item(row, col_sentido)

                if item:
                    if item.text() == "12":
                        item.setText("21")
                    else:
                        item.setText("12")
        except:
            pass
        self.status_message.emit("Line direction reversed!", 0)
   
    def get_closest_head(self, last_point, next_line_geom):
        """Decide whether the next line should start at end A or B, based on the vessel's current position."""
        pts = next_line_geom.asPolyline()
        dist_a = last_point.distance(pts[0])
        dist_b = last_point.distance(pts[-1])
        if dist_b < dist_a:
            return True 
        return False

    def start_monitoring(self):
        self.timer.start()
    
    def stop_monitoring(self):
        self.timer.stop()
        if self.map_tool:
            try: self.canvas.unsetMapTool(self.map_tool)
            except: pass

    def _on_canvas_clicked(self, point, button):
        if self.start_point is None:
            self.start_point = point
            self._show_marker(point, QColor("lime"))
        else:
            self.end_point = point
            self._show_marker(point, QColor("red"))
            QTimer.singleShot(0, self._create_ref_line_layer)
            self.canvas.unsetMapTool(self.map_tool)

    def _show_marker(self, point, color):
        if not hasattr(self, 'markers'):
            self.markers = []

        m = QgsVertexMarker(self.canvas)
        m.setCenter(point)
        m.setColor(color)
        m.setIconType(QgsVertexMarker.ICON_X)
        m.setPenWidth(3)
        self.markers.append(m)

    def _clear_markers(self):
        if hasattr(self, 'markers'):
            for m in self.markers:
                self.canvas.scene().removeItem(m)
            self.markers.clear()

    def _create_ref_line_layer(self):
        if not self.start_point or not self.end_point:
            return

        try:
            crs = QgsProject.instance().crs().authid()
            if not crs:
                crs = "EPSG:4326"
            layer_name = "GeoSea_Reference_Line"
            layers = QgsProject.instance().mapLayersByName(layer_name)
            
            if layers:
                vl = layers[0]
                pr = vl.dataProvider()
                
                ids_antigos = [f.id() for f in vl.getFeatures()]
                if ids_antigos:
                    pr.deleteFeatures(ids_antigos)
            else:
                vl = QgsVectorLayer(f"LineString?crs={crs}", layer_name, "memory")
                
                if not vl.isValid():
                    self.iface.messageBar().pushMessage("GeoSea", "Error: The layer is not valid for addition.", level=2)
                    return
                
                pr = vl.dataProvider()
                QgsProject.instance().addMapLayer(vl)

            # Creates the new line
            feat = QgsFeature()
            geom = QgsGeometry.fromPolylineXY([self.start_point, self.end_point])
            self.ref_geometry = geom 
            feat.setGeometry(geom)
            
            # Inserts into the temporary layer
            pr.addFeatures([feat])
            vl.updateExtents()
            vl.triggerRepaint()
            self.iface.setActiveLayer(vl)
            vl.selectAll()
            
            self.iface.messageBar().pushMessage("GeoSea", "Reference line created!", level=0)

            if hasattr(self, 'layers_updated'):
                self.layers_updated.emit()

        except Exception as e:
            print(f"Error creating row: {e}")
            self.iface.messageBar().pushMessage("GeoSea Erro", str(e), level=2)

        finally:
            self.start_point = None
            self.end_point = None
            if hasattr(self, '_clear_markers'):
                self._clear_markers()

    def _create_parallel_geometry(self, geometry, offset):
        pts = geometry.asPolyline()
        if len(pts) < 2: return None
        p1, p2 = pts[0], pts[-1]
        dx, dy = p2.x() - p1.x(), p2.y() - p1.y()
        dist = math.hypot(dx, dy)
        if dist == 0: return None
        nx, ny = -dy/dist, dx/dist 
        np1 = QgsPointXY(p1.x() + nx * offset, p1.y() + ny * offset)
        np2 = QgsPointXY(p2.x() + nx * offset, p2.y() + ny * offset)
        return QgsGeometry.fromPolylineXY([np1, np2])

    def _check_route_logic(self):
        pass

    def close_connections_on_shutdown(self):
        if self.timer and self.timer.isActive():
            self.timer.stop()
            
        try:
            QgsProject.instance().layersAdded.disconnect(self.layers_updated.emit)
            QgsProject.instance().layersRemoved.disconnect(self.layers_updated.emit)
        except TypeError:
            pass 