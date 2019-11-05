#ifndef MULTILAYER_LASER_SCAN_TIMEOFFSETS_AFTER_H
#define MULTILAYER_LASER_SCAN_TIMEOFFSETS_AFTER_H

namespace sensor_msgs
{
typedef multilayer_laser_scan::TimeOffsets TimeOffsets;
typedef multilayer_laser_scan::TimeOffsetsPtr TimeOffsetsPtr;
typedef multilayer_laser_scan::TimeOffsetsConstPtr TimeOffsetsConstPtr;
}

namespace multilayer_laser_scan
{

inline bool operator==(const TimeOffsets& lhs, const TimeOffsets& rhs)
{
  if (lhs.regular != rhs.regular)
    return false;

  if (lhs.regular)
  {
    if (lhs.increment != rhs.increment) return false;
    if (lhs.base_offset != rhs.base_offset) return false;
  }
  else
  {
    if (lhs.offsets != rhs.offsets) return false;
  }

  return true;
}

inline bool operator!=(const TimeOffsets& lhs, const TimeOffsets& rhs)
{
  return !(lhs == rhs);
}

}

#endif //MULTILAYER_LASER_SCAN_TIMEOFFSETS_AFTER_H
