# Data and layout of points with multiple channels of various datatypes.

# Describes custom channels and their layout in the binary data blob.
sensor_msgs/PointField[] fields

bool is_bigendian     # Are data bigendian?
uint32  point_step    # Length of a point in bytes
uint8[] data          # Actual point data, size is (point_step*number of points)