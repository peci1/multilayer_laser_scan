# Single full scan from a multi-layer laser range-finder.
#
# One could think about packing multiple LaserScan messages into an array,
# but that has several caveats. The biggest one is that most multi-layer
# scanners do not scan in planes but in cones, and LaserScan can only represent
# planes correctly.
#
# Most current multi-layer scanner drivers directly publish point clouds.
# This message type should be used to save bandwidth and retain information that
# usually gets lost when converting to a point cloud. As with the single-layer
# LaserScan, there is a deterministic way how a point cloud can be constructed
# from this laser scan.
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data
#
# The basic idea is that it is a 2D scan which consists of several
# rays/layers/rings/subscans which rotate around a (usually vertical) axis
# to create a panoramic multi-layer range-measurement. It might be similar to
# a depth image, but in this case we assume each point can have different
# timestamp.
#
# Practically, the laser is assumed to make several subscans between/during
# which it rotates with a constant angular velocity around its internal
# (usually vertical) axis. And during each subscan, it moves or rotates
# in a second dimension (usually vertical) to capture the individual rays.


# timestamp in the header is the canonical time of the scan from which all
# time offsets are computed
#
# in frame frame_id, horizontal angles are measured around the positive Z axis
# (counterclockwise, if Z is up) with zero angle being forward along the x axis
# vertical angles are measured around the negative Z axis (if Z points down,
# vertical angle is positive towards the up direction)
Header header

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

# Layout of the subscans. Angular and time offsets of individual rays.
# Corresponds to e.g. the ring (vertical) layout on a Velodyne.
ScanLayout subscan_layout

# Layout of the scan. Angular and time offsets of subscans.
# Corresponds to e.g. the rotational plane on a Velodyne.
# The layout can either describe the whole scan offsets,
# or it can
ScanLayout scan_layout

# Usually, the scanner makes its rotation motion even during capturing the
# subscan. These offsets are angles in the rotational (horizontal) plane
# that should be added to the rotational plane angle depending on which
# subscan is currently being captured. In most cases, this will either
# correspond to a constant rotational speed, or be zeros (for solid-state
# lidars).
# Obviously, the number of offsets in this field should be equal to the number
# of offsets in subscan layout.
# In case the subscan layout has regular angular offsets, this field can be
# filled e.g. so that min = 0, samples = subscan_layout.angular_offsets.samples,
# max = subscan_layout.angular_offsets.increment * samples
AngularOffsets scan_offsets_during_subscan

float32[] ranges      # range data [m] (Note: values < range_min or > range_max
                      # should be discarded)
float32[] intensities # intensity data [device-specific units].  If your
                      # device does not provide intensities, please leave
                      # the array empty.

# This can be e.g. the second values from dual-output scans (strongest + latest),
# ring number, estimated reflectivity, noise etc.
PointData custom_data    # data of points belonging to custom fields