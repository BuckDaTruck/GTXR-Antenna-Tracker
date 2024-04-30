import serial
import pynmea2
import numpy as np
import math
import threading
import time
EQUATORIAL_RADIUS_FEET = 20925721.785
POLAR_RADIUS_FEET = 20855567.585


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
                #print(location)
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

            #print("Results:")
            if dist_m is not None:
                global Distances
                Distances = dist_m
                #print(f"Distance: {dist_m:.3f} miles")
            if azimuth_deg is not None:
                global Azimuth
                Azimuth = azimuth_deg
                #print(f"Azimuth: {azimuth_deg:.4f}°")
            if altitude_deg is not None:
                global Altitude
                Altitude = altitude_deg
                #print(f"Altitude: {altitude_deg:.4f}°")
            return Distances, Azimuth, Altitude


def read_gps_data(
    port="/dev/tty.usbserial-B000JBMB", baudrate=4800, B_coords=(0, 0, 0)
):
    lat_B, long_B, alt_B = B_coords
    tracker="/dev/tty.usbmodem213301"
    baudrate2=115200
    # Open the serial port
    with serial.Serial(port, baudrate=baudrate, timeout=1) as ser:
        with serial.Serial(tracker, baudrate=baudrate2, timeout=1) as ser2:
            command_thread = None  # Placeholder for the command sending thread
            while True:
                # Read a line from the GPS output
                line = ser.readline().decode("ascii", errors="replace")

                # Try to parse the line as a GPGGA sentence
                try:
                    
                    msg = pynmea2.parse(line)
                    if isinstance(msg, pynmea2.types.talker.GGA):
                        # Extract latitude, longitude, and altitude
                        lat_A = float(msg.latitude)
                        long_A = float(msg.longitude)
                        alt_A = float(msg.altitude)

                        # Call the calculate function with the B coordinates
                        # Print the values for debugging
                        Distance,Azimuth, Altitude =calculate(lat_B, long_B, alt_B, lat_A, long_A, alt_A)
                        print(
                            f"lat_A: {lat_A}, long_A: {long_A}, alt_A: {alt_A}, lat_B: {lat_B}, long_B: {long_B}, alt_B: {alt_B},Azimuth: {Azimuth}°, Altitude: {Altitude}°, Distance: {Distance:.3f} miles"
                        )
                        
                        #ser2.write(f"M{Azimuth} {Altitude}".encode())
                        if Distance is not None and Azimuth is not None and Altitude is not None:
                            if command_thread is None or not command_thread.is_alive():
                                command = f"M{int(Azimuth)} {int(Altitude)}"
                                command_thread = threading.Thread(target=send_command, args=(ser2, command))
                                command_thread.start()
                                print(f"Sending command: {command}")
                    #

                    # print(f"Distance between A and B: {distance_2D} units")
                    # print(f"Vertical Distance: {vertical_distance} units")
                except pynmea2.ParseError:
                    # Ignore lines that can't be parsed
                    continue
def send_command(ser_out, command):
    while True:
        ser_out.write(command.encode('ascii') + b"\n")
        time.sleep(2)  # 

def set_B_coordinates():

    lat_B = 33.77216364445008
    long_B = -84.39563955416682
    alt_B = 295

    return (lat_B, long_B, alt_B)


if __name__ == "__main__":
    B_coords = set_B_coordinates()
    read_gps_data(port="/dev/tty.usbserial-B000JBMB", baudrate=4800, B_coords=B_coords)
