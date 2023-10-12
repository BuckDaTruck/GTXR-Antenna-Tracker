import wx
import serial.tools.list_ports
import math
import serial
import struct
import threading

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
stat = bytes("@ GPS_STAT", encoding="latin-1")
# Define the 'com' list at the global scope
com = ["Select Port"]

def parse_angle(angle_str, limit):
    """
    Parses an angle string and returns the angle as a float if it is within the specified limit.

    Args:
        angle_str (str): The angle string to parse.
        limit (float): The limit within which the angle must be.

    Returns:
        float or None: The parsed angle as a float if it is within the limit, or None if it is not.
    """
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
    """
    Parses the given elevation string and returns the elevation in feet.

    Args:
        elevation_str (str): The string representation of the elevation.

    Returns:
        float: The elevation in feet if the string is a valid numeric value, otherwise None.
    """
    try:
        elevation_feet = float(elevation_str)
        if math.isnan(elevation_feet):
            print("Invalid elevation value.")
            return None
        else:
            return elevation_feet
    except ValueError:
        print("Invalid elevation value. Please enter a numeric value.")
        return None

def parse_location(lati, long, alt):
    """
    Parses the latitude, longitude, and altitude strings and returns a dictionary containing the parsed values.

    Args:
        lati (str): The latitude string.
        long (str): The longitude string.
        alt (str): The altitude string.

    Returns:
        dict: A dictionary containing the parsed latitude, longitude, and altitude values. Returns None if any of the strings cannot be parsed.
    """
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
    """
    Calculates the radius of the earth in feet at a given latitude.

    Args:
        latitude_radians (float): The latitude in radians.

    Returns:
        float: The radius of the earth in feet at the given latitude.
    """
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
    """
    Calculates the geocentric latitude from the given geodetic latitude.

    Args:
        lat (float): The geodetic latitude in radians.

    Returns:
        float: The geocentric latitude in radians.
    """
    e2 = 0.00669437999014
    clat = math.atan((1.0 - e2) * math.tan(lat))
    return clat

def location_to_point(c):
    """
    Converts a location on Earth to a point in 3D space.

    Args:
        c (dict): A dictionary containing the latitude, longitude, and elevation of the location.

    Returns:
        dict: A dictionary containing the x, y, and z coordinates of the point, as well as the radius of the Earth at that location and the normal vector of the point.
    """
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
    """
    Calculates the Euclidean distance between two points in 3D space.

    Args:
        ap (dict): A dictionary representing the coordinates of the first point.
        bp (dict): A dictionary representing the coordinates of the second point.

    Returns:
        float: The Euclidean distance between the two points.
    """
    dx = ap['x'] - bp['x']
    dy = ap['y'] - bp['y']
    dz = ap['z'] - bp['z']
    return math.sqrt(dx * dx + dy * dy + dz * dz)

def rotate_globe(b, a, bradius, aradius):
    """
    Rotate a point on the surface of a sphere around another point on the surface of the same sphere.

    Args:
        b (dict): A dictionary representing the point to be rotated, with keys 'lat', 'lon', and 'elv'.
        a (dict): A dictionary representing the point around which to rotate, with keys 'lat', 'lon', and 'elv'.
        bradius (float): The radius of the sphere at the point to be rotated.
        aradius (float): The radius of the sphere at the point around which to rotate.

    Returns:
        dict: A dictionary representing the rotated point, with keys 'x', 'y', 'z', and 'radius'.
    """
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
    """
    Calculates the normalized vector difference between two points in 3D space.

    Args:
        b (dict): A dictionary representing the second point in 3D space with keys 'x', 'y', and 'z'.
        a (dict): A dictionary representing the first point in 3D space with keys 'x', 'y', and 'z'.

    Returns:
        dict: A dictionary representing the normalized vector difference between the two points with keys 'x', 'y', 'z', and 'radius'.
              The 'x', 'y', and 'z' keys represent the normalized vector components, and the 'radius' key represents the magnitude of the vector.
              Returns None if the two points are the same.
    """
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
    """
    Lists all available serial ports and returns a list of their names.

    Returns:
        list: A list of available serial ports.
    """
    global com
    ports = list(serial.tools.list_ports.comports())
    com = ["Select Port"]
    for i, port in enumerate(ports):
        print(f"{i + 1}: Serial Port: {port.device}, Description: {port.description}")
        com.append(f"{i + 1}: Serial Port: {port.device}, Description: {port.description}")
    return com  # Return the list of available serial ports

class Example(wx.Frame):
    """
    A class that represents the main window of the Antenna Tracker application.

    Attributes:
    alt_A (int): The altitude of point A.
    lat_A (int): The latitude of point A.
    long_A (int): The longitude of point A.
    alt_B (int): The altitude of point B.
    lat_B (int): The latitude of point B.
    long_B (int): The longitude of point B.
    ser (serial.Serial): The serial port object for the ground station.
    ser_feather (serial.Serial): The serial port object for the Featherweight GPS.
    com (list): A list of available serial ports.
    """

    def __init__(self, ser, ser_feather, com, *args, **kw):
        """
        Initializes the Example object.

        Parameters:
        ser (serial.Serial): The serial port object for the ground station.
        ser_feather (serial.Serial): The serial port object for the Featherweight GPS.
        com (list): A list of available serial ports.
        *args: Variable length argument list.
        **kw: Arbitrary keyword arguments.
        """
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
        """
        Initializes the user interface of the Antenna Tracker application.
        """
        # UI code goes here
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
        
        # Rescan for serial ports


    """
    This module contains a class that initializes the user interface for an Antenna Tracker application. 
    The UI includes various widgets such as ComboBox, CheckBox, StaticText, TextCtrl, and Button. 
    The class also defines methods to handle user interactions with the widgets, such as selecting a serial port, 
    checking/unchecking a checkbox, entering text in a TextCtrl, clicking a button, and closing the application. 
    """
    def InitUI(self):
        font = wx.Font(24, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        pnl = wx.Panel(self)
       
        
        newport()
        self.Ground = wx.ComboBox(pnl, pos=(20, 30), choices=self.com, style=wx.CB_READONLY)
        self.Ground.Bind(wx.EVT_COMBOBOX, self.select_serial_port)

        self.Feather = wx.ComboBox(pnl, pos=(20, 80), choices=self.com, style=wx.CB_READONLY)
        self.Feather.Bind(wx.EVT_COMBOBOX, self.OnFeather)
        self.staticA = wx.CheckBox(pnl, label='Static', pos=(150, 10))
        self.staticA.Bind(wx.EVT_CHECKBOX, self.OnStaticA)
        self.staticB = wx.CheckBox(pnl, label='Static', pos=(150, 60))
        self.staticB.Bind(wx.EVT_CHECKBOX, self.OnStaticB)
       

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
        self.RescanButton = wx.Button(self, label="Rescan", pos=(290, 340))
        self.Bind(wx.EVT_BUTTON, self.rescan_serial_ports, self.RescanButton)


        self.Centre()
        self.Show(True)
        btn.Bind(wx.EVT_BUTTON, self.OnClose)

    def on_button_click(self, event):
        wx.MessageBox("Coded By Buckley Wiley\nbuckley@buckleywiley.com\nGTXR Antena Tracker V1.0", "About")
   
    def on_home_click(self, event):
        print("Home")
    def rescan_serial_ports(self, event):
        available_ports = newport()  # Get the list of available serial ports

        # Update the list of available serial ports in the GUI
        self.serial_port_choice.Clear()
        self.serial_port_choice.AppendItems(available_ports)

        # If the currently selected serial port is still available, select it
        if self.com in available_ports:
            self.serial_port_choice.SetStringSelection(self.com)
        else:
            self.com = None  # Clear the selected serial port if it is no longer available
    def parse_data(self):
            """
            Parses the serial data received from the GPS module and returns a dictionary containing the parsed data.

            Returns:
            dict: A dictionary containing the parsed data with the following keys:
                - time: The time in the format HH:MM:SS.mmm.
                - alt: The altitude in meters.
                - lat: The latitude in decimal degrees.
                - long: The longitude in decimal degrees.
                - hvel: The horizontal velocity in meters per second.
                - hhead: The horizontal heading in degrees.
                - uvel: The vertical velocity in meters per second.
            """
            # If the serial data starts with "@ GPS_STAT", parse the data
            self.data = ""
            self.stat = "@ GPS_STAT"
            parsed_data = {}

            if self.data.startswith(self.stat):
                timeindex = self.data.index(':')
                altindex = self.data.index('Alt')
                latindex = self.data.index('+')
                longindex = self.data.index('ln')
                velindex = self.data.index('Vel')

                parsed_data['time'] = self.data[timeindex - 2: timeindex + 9]
                parsed_data['alt'] = self.data[altindex + 4: latindex - 4]
                parsed_data['lat'] = self.data[latindex: longindex - 1]
                parsed_data['long'] = self.data[longindex + 3: velindex - 1]
                parsed_data['hvel'] = self.data[velindex + 4:velindex + 9]
                parsed_data['hhead'] = self.data[velindex + 10: velindex + 14]
                parsed_data['uvel'] = self.data[velindex + 15: velindex + 20]
                #print("Alt", self.alt, "Lat", self.lat, "Long", self.long, "Hvel", self.hvel, "Hhead", self.hhead, "Uvel", self.uvel)
                return parsed_data

            
    def update_calculations(self):
        """
        Updates the calculations for the antenna tracker based on the current values of the GUI inputs.
        Parses the data and retrieves the latitude, longitude, and altitude values for both antennas.
        Calculates the distance, azimuth, and altitude between the two antennas.
        Converts the azimuth and altitude values to bytes and sends them over serial.
        """
        parsed_data = self.parse_data()

        self.stat_alt_B = self.Altitude_textB.GetValue()   
        self.stat_lat_B = self.latitude_textB.GetValue() 
        self.stat_long_B = self.longitude_textB.GetValue() 
        self.stat_alt_A = self.Altitude_textA.GetValue()  
        self.stat_lat_A = self.latitude_textA.GetValue() 
        self.stat_long_A = self.longitude_textA.GetValue()
        if self.statA:
            self.alt_A = self.stat_alt_A
            self.lat_A = self.stat_lat_A
            self.long_A = self.stat_long_A
        else:
            self.alt_A = 0
            self.lat_A = 0
            self.long_A = 0
        if self.statB:
            self.alt_B = self.stat_alt_B
            self.lat_B = self.stat_lat_B
            self.long_B = self.stat_long_B
        else:
            self.alt_B = parsed_data['alt']
            print(parsed_data['alt'])
            self.lat_B = parsed_data['lat']
            print(parsed_data['lat'])
            self.long_B = parsed_data['long']
            print(parsed_data['long'])
        alt_A = self.alt_A
        lat_A = self.lat_A
        long_A = self.long_A
        alt_B = self.alt_B
        lat_B = self.lat_B
        long_B = self.long_B
        calculate(lat_A, long_A, alt_A, lat_B, long_B, alt_B)
        self.DistanceLabel.SetLabel(f"Distance: {Distances:.3f} Miles")
        self.AzimuthLabel.SetLabel(f"Azimuth: {Azimuth:.4f}°")
        self.AltitudeLabel.SetLabel(f"Altitude: {Altitude:.4f}°")
        azimuth_byte = Azimuth
        altitude_byte = Altitude
        # Convert float values to bytes
        Azimuth_bytes = struct.pack('f',  azimuth_byte)
        Altitude_bytes = struct.pack('f', altitude_byte)

        # Send the bytes over serial
        self.ser.write(Altitude_bytes)
        self.ser.write(Azimuth_bytes)

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

    def read_serial_data(self):
        """
        Reads the serial data from the Featherweight GPS module and updates the GUI with the new data.
        """
        while self.ser_feather.is_open:
            try:
                # Read the serial data
                data = self.ser_feather.readline().decode('utf-8').strip()
                # Parse the data
                parsed_data = self.parse_data(data)
                # Update the GUI with the new data
                self.update_gui(parsed_data)
            except serial.SerialException:
                print("Serial port closed.")
                break

    def open_serial_port_feather(self):
        """
        Opens the serial port for communication with the Featherweight GPS module.
        """
        try:
            self.ser_feather = serial.Serial(self.selected_serial_port_feather, 115200)
            print(f"Opened Serial Port (Featherweight GPS): {self.selected_serial_port_feather}")
            # Start a new thread to read the serial data in the background
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
        except serial.SerialException:
            print("Failed to open serial port.")

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

    def select_serial_port(self, event):
        """
        Handles the selection of a serial port for communication with the Featherweight GPS module.
        """
        selected_index = event.GetSelection()
        if selected_index > 0:
            ports = list(serial.tools.list_ports.comports())
            selected_port_info_feather = ports[selected_index - 1]
            self.selected_serial_port_feather = selected_port_info_feather.device
            self.open_serial_port_feather()
            print(f"Selected Serial Port (Featherweight GPS): {self.selected_serial_port_feather}")
            self.rescan_serial_ports(event)  # Call the rescan_serial_ports method

    def OnClose(self, e):
        if self.ser.is_open:
            self.ser.close()
        self.Close(True)

def main():
    """
    Initializes serial ports for communication with the antenna tracker and Feather board, 
    gets the list of available serial ports, and starts the GUI application.
    """
    selected_serial_port = None
    selected_serial_port_feather = None  # Initialize Feather serial port
    ser = serial.Serial(selected_serial_port, 115200)
    ser_feather = serial.Serial(selected_serial_port_feather, 115200)  # Pass Feather serial port
    com = newport()  # Get the list of available serial ports
    ex = wx.App()
    Example(ser, ser_feather, com, None)
    ex.MainLoop()
    

if __name__ == '__main__':
    main()
