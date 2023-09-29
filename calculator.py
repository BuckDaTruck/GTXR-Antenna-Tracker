import wx
import serial.tools.list_ports
import math
import serial
import struct

# Define constants for radius values
EQUATORIAL_RADIUS_FEET = 20925721.785
POLAR_RADIUS_FEET = 20855567.585

# Initialize status variables
statA = False
statB = False

# Initialize global variables
Distances = 0.0
Altitude = 0.0
Azimuth = 0.0

alt_A = 0
lat_A = 0
long_A = 0
alt_B = 0
lat_B = 0
long_B = 0

# Initialize status variables
stat_alt_A = 0
stat_lat_A = 0
stat_long_A = 0
stat_alt_B = 0
stat_lat_B = 0
stat_long_B = 0
statA = False
statB = False

# Initialize the 'sat' list outside the 'newport' function
sat = ["none"]

# Define the 'com' list at the global scope
com = ["Select Port"]

import tkinter as tk
import math

def parse_angle(angle_str, limit):
    try:
        angle = float(angle_str)
        if math.isnan(angle) or (angle < -limit) or (angle > limit):
            print("Invalid angle value.")
            return None
        else:
            return angle
    except ValueError:
        print("Invalid angle value. Please enter a numeric value.")
        return None
def parse_elevation(elevation_str):
    try:
        elevation_feet = float(elevation_str)
        if math.isnan(elevation_feet):
            print("Invalid elevation value.")
            return None
        else:
         
            return  elevation_feet
    except ValueError:
        print("Invalid elevation value. Please enter a numeric value.")
        return None

def parse_location(lati, long, alt):
    lat_str = lati
    lon_str = long
    elv_str = alt

    lat = parse_angle(lat_str, 90.0)
    if lat is not None:
        lon = parse_angle(lon_str, 180.0)
        if lon is not None:
            elv = parse_elevation(elv_str)
            if elv is not None:
                location = {'lat': lat, 'lon': lon, 'elv': elv}
                print(location)
                return location
    return None

def earth_radius_in_feet(latitude_radians):
    a = 20925721.785  # equatorial radius in feet
    b = 20855567.585  # polar radius in feet
    cos = math.cos(latitude_radians)
    sin = math.sin(latitude_radians)
    t1 = a * a * cos
    t2 = b * b * sin
    t3 = a * cos
    t4 = b * sin
    return math.sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4))

def geocentric_latitude(lat):
    e2 = 0.00669437999014
    clat = math.atan((1.0 - e2) * math.tan(lat))
    return clat

def location_to_point(c):
    lat = c['lat'] * math.pi / 180.0
    lon = c['lon'] * math.pi / 180.0
    radius = earth_radius_in_feet(lat)
    clat = geocentric_latitude(lat)
    cos_lon = math.cos(lon)
    sin_lon = math.sin(lon)
    cos_lat = math.cos(clat)
    sin_lat = math.sin(clat)
    x = radius * cos_lon * cos_lat
    y = radius * sin_lon * cos_lat
    z = radius * sin_lat
    cos_glat = math.cos(lat)
    sin_glat = math.sin(lat)
    nx = cos_glat * cos_lon
    ny = cos_glat * sin_lon
    nz = sin_glat
    x += c['elv'] * nx
    y += c['elv'] * ny
    z += c['elv'] * nz
    return {'x': x, 'y': y, 'z': z, 'radius': radius, 'nx': nx, 'ny': ny, 'nz': nz}

def distance(ap, bp):
    dx = ap['x'] - bp['x']
    dy = ap['y'] - bp['y']
    dz = ap['z'] - bp['z']
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def rotate_globe(b, a, bradius, aradius):
    br = {'lat': b['lat'], 'lon': (b['lon'] - a['lon']), 'elv': b['elv']}
    brp = location_to_point(br)
    alat = geocentric_latitude(-a['lat'] * math.pi / 180.0)
    acos = math.cos(alat)
    asin = math.sin(alat)
    bx = (brp['x'] * acos) - (brp['z'] * asin)
    by = brp['y']
    bz = (brp['x'] * asin) + (brp['z'] * acos)
    return {'x': bx, 'y': by, 'z': bz, 'radius': bradius}

def normalize_vector_diff(b, a):
    dx = b['x'] - a['x']
    dy = b['y'] - a['y']
    dz = b['z'] - a['z']
    dist2 = dx * dx + dy * dy + dz * dz
    if dist2 == 0:
        return None
    dist = math.sqrt(dist2)
    return {'x': (dx / dist), 'y': (dy / dist), 'z': (dz / dist), 'radius': 1.0}

def calculate(lat_A, long_A, alt_A, lat_B, long_B, alt_B):
    a = parse_location(lat_A, long_A, alt_A)
    if a is not None:
        b = parse_location(lat_B, long_B, alt_B)
        if b is not None:
            ap = location_to_point(a)
            bp = location_to_point(b)
            dist_m = 0.000189394 * distance(ap, bp)
            azimuth_deg = None
            altitude_deg = None

            # Calculate azimuth
            br = rotate_globe(b, a, bp['radius'], ap['radius'])
            if br['z'] * br['z'] + br['y'] * br['y'] > 1.0e-6:
                theta = math.atan2(br['z'], br['y']) * 180.0 / math.pi
                azimuth = 90.0 - theta
                if azimuth < 0.0:
                    azimuth += 360.0
                azimuth_deg = azimuth

            # Calculate altitude
            bma = normalize_vector_diff(bp, ap)
            if bma is not None:
                altitude = 90.0 - (180.0 / math.pi) * math.acos(bma['x'] * ap['nx'] + bma['y'] * ap['ny'] + bma['z'] * ap['nz'])
                altitude_deg = altitude

            print("Results:")
            if dist_m is not None:
                global Distances
                Distances = dist_m
                print(f"Distance: {dist_m:.3f} miles")
            if azimuth_deg is not None:
                global Azimuth
                Azimuth = azimuth_deg
                print(f"Azimuth: {azimuth_deg:.4f}°")
            if altitude_deg is not None:
                global Altitude
                Altitude = altitude_deg
                print(f"Altitude: {altitude_deg:.4f}°")




def newport():
    global com
    ports = list(serial.tools.list_ports.comports())
    com = ["Select Port"]
    for i, port in enumerate(ports):
        print(f"{i + 1}: Serial Port: {port.device}, Description: {port.description}")
        com.append(f"{i + 1}: Serial Port: {port.device}, Description: {port.description}")
    return com  # Return the list of available serial ports

class Example(wx.Frame):
    def __init__(self, ser, ser_feather, com, *args, **kw):
        super(Example, self).__init__(*args, **kw)
        self.alt_A = 0
        self.lat_A = 0
        self.long_A = 0
        self.alt_B = 0
        self.lat_B = 0
        self.long_B = 0
        self.ser = ser
        self.ser_feather = ser_feather
        self.com = com

        self.InitUI()

    def InitUI(self):
        font = wx.Font(24, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        pnl = wx.Panel(self)

        newport()
        Ground = wx.ComboBox(pnl, pos=(20, 30), choices=com, style=wx.CB_READONLY)
        Ground.Bind(wx.EVT_COMBOBOX, self.select_serial_port)

        Feather = wx.ComboBox(pnl, pos=(20, 80), choices=com, style=wx.CB_READONLY)
        self.staticA = wx.CheckBox(pnl, label='Static', pos=(150, 10))
        self.staticA.Bind(wx.EVT_CHECKBOX, self.OnStaticA)
        self.staticB = wx.CheckBox(pnl, label='Static', pos=(150, 60))
        self.staticB.Bind(wx.EVT_CHECKBOX, self.OnStaticB)
        Feather.Bind(wx.EVT_COMBOBOX, self.OnFeather)

        self.Ground = wx.StaticText(pnl, label='', pos=(20, 140))
        self.Feather = wx.StaticText(pnl, label='', pos=(20, 160))
        self.SetSize((800, 400))
        self.SetTitle('Antenna Tracker')

        wx.StaticText(self, label='Ground Station', pos=(25, 10))
        wx.StaticText(pnl, label="Lat:", pos=(220, 10))
        wx.StaticText(pnl, label="Long:", pos=(400, 10))
        wx.StaticText(pnl, label="Elv:", pos=(580, 10))
        self.longitude_textA = wx.TextCtrl(pnl, pos=(440, 6), size=(120, -1))
        self.longitude_textA.SetHint("Enter Longitude")
        self.latitude_textA = wx.TextCtrl(pnl, pos=(260, 6), size=(120, -1))
        self.latitude_textA.SetHint("Enter Latitude")
        self.Altitude_textA = wx.TextCtrl(pnl, pos=(610, 6), size=(120, -1))
        self.Altitude_textA.SetHint("Enter Altitude")

        wx.StaticText(self, label='Featherweight GPS', pos=(25, 60))
        wx.StaticText(pnl, label="Lat:", pos=(220, 60))
        wx.StaticText(pnl, label="Long:", pos=(400, 60))
        wx.StaticText(pnl, label="Elv:", pos=(580, 60))
        self.longitude_textB = wx.TextCtrl(pnl, pos=(440, 56), size=(120, -1))
        self.longitude_textB.SetHint("Enter Longitude")
        self.latitude_textB = wx.TextCtrl(pnl, pos=(260, 56), size=(120, -1))
        self.latitude_textB.SetHint("Enter Latitude")
        self.Altitude_textB = wx.TextCtrl(pnl, pos=(610, 56), size=(120, -1))
        self.Altitude_textB.SetHint("Enter Elavation")
        self.latitude_textA.Bind(wx.EVT_TEXT, self.OnInputChange)
        self.longitude_textA.Bind(wx.EVT_TEXT, self.OnInputChange)
        self.Altitude_textA.Bind(wx.EVT_TEXT, self.OnInputChange)
        self.latitude_textB.Bind(wx.EVT_TEXT, self.OnInputChange)
        self.longitude_textB.Bind(wx.EVT_TEXT, self.OnInputChange)
        self.Altitude_textB.Bind(wx.EVT_TEXT, self.OnInputChange)

        self.GroundStation = wx.StaticText(pnl, label='', pos=(540, 280))
        self.DistanceLabel = wx.StaticText(pnl, label='', pos=(540, 280))
        self.DistanceLabel.SetFont(font)
        self.AzimuthLabel = wx.StaticText(pnl, label='', pos=(540, 315))
        self.AzimuthLabel.SetFont(font)
        self.AltitudeLabel = wx.StaticText(pnl, label='', pos=(540, 350))
        self.AltitudeLabel.SetFont(font)
        self.About = wx.Button(self, label="About", pos=(110, 340))
        self.Bind(wx.EVT_BUTTON, self.on_button_click, self.About)
        btn = wx.Button(self, label='Exit', pos=(20, 340))
        self.Home = wx.Button(self, label="Home", pos=(200, 340))
        self.Bind(wx.EVT_BUTTON, self.on_home_click, self.Home)

        self.Centre()
        self.Show(True)
        btn.Bind(wx.EVT_BUTTON, self.OnClose)

    def on_button_click(self, event):
        wx.MessageBox("Coded By Buckley Wiley\nbuckley@buckleywiley.com\nGTXR Antena Tracker V1.0", "About")

    def on_home_click(self, event):
        print("Home")

    def update_calculations(self):
        # Calculate and update values here
        pass

    def open_serial_port(self):
        if self.selected_serial_port is not None:
            try:
                self.ser = serial.Serial(self.selected_serial_port, 115200)
                print("Serial port opened successfully")
            except Exception as e:
                print(f"Error opening serial port: {str(e)}")

    def OnInputChange(self, e):
        self.update_calculations()

    def OnGround(self, e):
        i = e.GetString()
        self.Ground.SetLabel(i)

    def select_serial_port(self, event):
        selected_index = event.GetSelection()
        if selected_index > 0:
            ports = list(serial.tools.list_ports.comports())
            selected_port_info = ports[selected_index - 1]
            self.selected_serial_port = selected_port_info.device
            self.open_serial_port()
            print(f"Selected Serial Port (Ground Station): {self.selected_serial_port}")

    def open_serial_port_feather(self):
        if self.selected_serial_port_feather is not None:
            try:
                self.ser_feather = serial.Serial(self.selected_serial_port_feather, 115200)
                print("Serial port for Featherweight GPS opened successfully")
            except Exception as e:
                print(f"Error opening serial port for Featherweight GPS: {str(e)}")

    def OnFeather(self, event):
        selected_index = event.GetSelection()
        if selected_index > 0:
            ports = list(serial.tools.list_ports.comports())
            selected_port_info_feather = ports[selected_index - 1]
            self.selected_serial_port_feather = selected_port_info_feather.device
            self.open_serial_port_feather()
            print(f"Selected Serial Port (Featherweight GPS): {self.selected_serial_port_feather}")

    def OnStaticA(self, e):
        self.longitude_textA.Enable(self.staticA.GetValue())
        self.latitude_textA.Enable(self.staticA.GetValue())
        self.Altitude_textA.Enable(self.staticA.GetValue())
        self.statA = self.staticA.GetValue()

    def OnStaticB(self, e):
        self.longitude_textB.Enable(self.staticB.GetValue())
        self.latitude_textB.Enable(self.staticB.GetValue())
        self.Altitude_textB.Enable(self.staticB.GetValue())
        self.statB = self.staticB.GetValue()

    def OnClose(self, e):
        if self.ser.is_open:
            self.ser.close()
        self.Close(True)


def main():
    selected_serial_port = None
    selected_serial_port_feather = None  # Initialize Feather serial port
    ser = serial.Serial(selected_serial_port, 115200)
    ser_feather = serial.Serial(selected_serial_port_feather, 115200)  # Pass Feather serial port
    com = newport()  # Get the list of available serial ports
    ex = wx.App()
    Example(ser, ser_feather, com, None)
    ex.MainLoop()
    calculate()


if __name__ == '__main__':
    main()

