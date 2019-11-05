# Time offsets of individual rays/subscans in one dimension of a mutlilayer laser scan.
# The offsets can be all zero, which means all rays are captured at the same time
# (e.g. in solid-state lidars).

# Whether time offsets of the rays/subscans can be computed as
# ray_number * increment + base_offset.
# If true, set increment, base_offset and samples.
# If false, fill offsets with the time offsets of individual rays/subscans.
bool regular

# if regular

duration base_offset  # the base offset to which all computed offsets will be added
duration increment # time between individual ray/subscan measurements

# else

# time offsets of the individual rays/subscans from their canonical timestamp
duration[] offsets

# endif