# Angular offsets of individual rays/subscans in one dimension of a mutlilayer laser scan.

# Whether angular offsets are represented as a regularly sampled circular sector.
# If true, set min, max, exclude_last and either increment or samples.
# If false, fill offsets with values denoting the offsets of individual
# rays from the horizontal/vertical plane of the given frame_id.
bool regular

# if regular

# If the scan is 360 degrees, the max endpoint is exclusive, otherwise it is
# inclusive. So for a 360 degrees scan, set min = -pi and max = pi.
# If min == max, only one ray will be generated regardless of increment or samples.
float64 min        # start angle of the scan [rad] range <-2pi, 2pi>
float64 max        # end angle of the scan [rad] range <-2pi, 2pi>

# if true, the last angle in the range is exclusive; this comes useful if you
# want to define an exact 2 pi range which doesn't include the zero point twice
bool exclude_last

# angular distance between measurements [rad] range <-2pi, 2pi>
# negative increment denotes inverse rotation of the scanner
# in case of negative increment, the scan begins at max and goes down to min
float64 increment

# if you don't want to compute increment manually but know how many samples
# should be measured, set this number and leave increment unfilled (0), the
# increment can be computed by dividing the angular range by the number of
# samples (taking into account if the endpoint is inclusive or exclusive)
# if this number is negative, it just means the scan should begin at max and
# go down to min (i.e. reverse its direction)
int32 samples

# in case both increment and samples are nonzero, their signs should be the same
# and the angles are computed based on increment; if the computed number of
# samples differs from the one in this message, a warning should be issued

# else

# angular offsets of individual rays/subscans from the frame_id's
# vertical/horizontal plane [rad]
float64[] offsets

# endif
