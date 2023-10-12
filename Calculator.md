parse_location: This function takes latitude, longitude, and altitude values as input and returns a dictionary with keys 'lat', 'long', and 'alt' representing the parsed location. If the input values are invalid, the function returns None.

location_to_point: This function takes a location dictionary as input and returns a dictionary with keys 'x', 'y', 'z', and 'radius' representing the Cartesian coordinates of the location on a unit sphere. The 'radius' key represents the radius of the Earth in meters.

distance: This function takes two point dictionaries as input and returns the great-circle distance between the two points in radians.

rotate_globe: This function takes two point dictionaries and their radii as input and returns a dictionary representing the normalized vector difference between the two points. The vector difference is calculated by rotating the Earth's surface so that the first point is at the origin and the second point is on the positive y-axis, and then calculating the vector difference between the two points.

normalize_vector_diff: This function takes two point dictionaries as input and returns a dictionary representing the normalized vector difference between the two points. The 'x', 'y', and 'z' keys represent the normalized vector components, and the 'radius' key represents the magnitude of the vector.

calculate: This function takes latitude, longitude, and altitude values for two locations as input and returns the distance, azimuth, and altitude between the two locations. The function first parses the input values into location dictionaries using the parse_location function. It then converts the location dictionaries into point dictionaries using the location_to_point function. The great-circle distance between the two points is calculated using the distance function and converted to meters. The azimuth and altitude between the two points are calculated using the rotate_globe function and the normalize_vector_diff function. The azimuth is returned in degrees clockwise from true north, and the altitude is returned in degrees above the horizon.