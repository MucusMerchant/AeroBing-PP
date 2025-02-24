import sys
import folium
from PyQt6 import QtWidgets, QtWebEngineWidgets, QtCore
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(current_dir, 'assets', 'lala.png')
import base64
with open(image_path, "rb") as img_file:
    base64_image = base64.b64encode(img_file.read()).decode('utf-8')

def find_map_variable(html):
    pattern = "var map_"

    starting_index = html.find(pattern) + 4
    tmp_html = html[starting_index:]
    ending_index = tmp_html.find(" =") + starting_index

    return html[starting_index:ending_index]

def find_line_variable(html):
    pattern = "var poly_line_"

    starting_index = html.find(pattern) + 4
    tmp_html = html[starting_index:]
    ending_index = tmp_html.find(" =") + starting_index

    return html[starting_index:ending_index]

def find_dot_variable(html):
    pattern = "var marker_"

    starting_index = html.find(pattern) + 4
    tmp_html = html[starting_index:]
    ending_index = tmp_html.find(" =") + starting_index

    return html[starting_index:ending_index]

class LiveMapWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live Map with Line")
        self.setGeometry(100, 100, 800, 600)
        

        self.layout = QtWidgets.QVBoxLayout(self)
        self.browser = QtWebEngineWidgets.QWebEngineView(self)

        self.layout.addWidget(self.browser)

        # Initialize folium map
        self.m = folium.Map(location=[45.5236, -122.6750], zoom_start=13)
        
        # Prepare initial line
        self.coordinates = [[45.5236, -122.6750]]
        self.polyline = folium.PolyLine(self.coordinates, color="blue", weight=5, opacity=0.7).add_to(self.m)
        #popup = folium.Popup(folium.Element(), sticky=True, max_width='10%')
        self.marker = folium.Marker([45.5236, -122.6750]).add_to(self.m)

        # Render the map
        html = self.m.get_root().render()
        print(html)
        self.browser.setHtml(html)
        self.map_name = find_map_variable(html)
        self.line_name = find_line_variable(html)
        self.dot_name = find_dot_variable(html)

       
        self.show()


        # Timer for simulating stream of coordinates
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_line)
        self.timer.start(1000)  # Update every second

    def update_line(self):
        # Simulate new coordinates (you can replace this with real-time data)
        new_coord = [45.5236 + 0.1, -122.6750 + 0.1]  # Simulated new coordinate
        self.coordinates.append(new_coord)
        
        js_code = '''
        (%s).addLatLng([%d,%d]);
        (%s).setLatLng([%d,%d]).openPopup();
        (%s).setView([%d, %d]);
        ''' %(self.line_name, new_coord[0], new_coord[1], self.dot_name, new_coord[0], new_coord[1], self.map_name, new_coord[0], new_coord[1])
        
        # Inject the JavaScript code to update the polyline
        self.browser.page().runJavaScript(js_code)
        
        self.browser.page().runJavaScript(f"""
            var base64Image = 'data:image/png;base64,{base64_image}';
            {self.dot_name}.setIcon(L.icon({{'iconUrl':base64Image}}));""")
    

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = LiveMapWindow()
    sys.exit(app.exec())
