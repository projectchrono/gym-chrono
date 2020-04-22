from pychrono import ChVectorD
# GPScoord is a wrapper class around ChVector
# Stores GPS points as (lat, long, alt)
# All coordinates are in degrees
class GPSCoord(ChVectorD):
    # Constructs a GPScoord with a default z value
    def __init__(self, x, y, z=0):
        ChVectorD.__init__(self, x, y, z)

    # Access to components
    # Primarily for cleaner and more understandable code
    @property
    def lat(self):
        return self.x
    @property
    def long(self):
        return self.y
    @property
    def alt(self):
        return self.z

    # Access to components with conversions
    @property
    def lat_rad(self):
        return math.radians(self.lat)
    @property
    def long_rad(self):
        return math.radians(self.long)
    @property
    def lat_cos(self):
        return math.cos(self.lat)

# Radius of the earth
EARTH_RADIUS = 6378.1e3  # [m]

# Origin being somewhere in Madison WI
origin = GPSCoord(43.070, -89.400, 260.0)

def toCartesian(coord):
    """ Approximation: Converts GPS coordinate to x,y,z provided some origin """

    # x is East, y is North
    x = EARTH_RADIUS * (coord.long_rad - origin.long_rad) * origin.lat_cos
    y = EARTH_RADIUS * (coord.lat_rad - origin.lat_rad)
    z = coord.alt - origin.alt

    return chrono.ChVectorD(x, y, z)


def toGPSCoordinate(pos):
    """ Approximation: Converts x,y,z to GPS Coordinate provided some origin """

    EARTH_RADIUS = 6378.1e3  # [m]

    # x is East, y is North
    long = math.degrees(pos.x / EARTH_RADIUS / origin.lat_cos + origin.long_rad)
    lat = math.degrees(pos.y / EARTH_RADIUS + origin.lat_rad)
    alt = pos.z + origin.alt

    return chrono.ChVectorD(lat, long, alt)
