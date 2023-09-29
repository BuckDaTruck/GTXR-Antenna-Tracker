
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

def parse_location(prefix):
    lat_str = input(f"Enter {prefix} Latitude (degrees): ")
    lon_str = input(f"Enter {prefix} Longitude (degrees): ")
    elv_str = input(f"Enter {prefix} Elevation (feet): ")

    lat = parse_angle(lat_str, 90.0)
    if lat is not None:
        lon = parse_angle(lon_str, 180.0)
        if lon is not None:
            elv = parse_elevation(elv_str)
            if elv is not None:
                location = {'lat': lat, 'lon': lon, 'elv': elv}
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

def calculate():
    a = parse_location('Point A')
    if a is not None:
        b = parse_location('Point B')
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
                print(f"Distance: {dist_m:.3f} miles")
       
            if azimuth_deg is not None:
                print(f"Azimuth: {azimuth_deg:.4f}°")
           
            if altitude_deg is not None:
                print(f"Altitude: {altitude_deg:.4f}°")


if __name__ == "__main__":
    calculate()
