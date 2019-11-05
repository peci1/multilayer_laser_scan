#ifndef MULTILAYER_LASER_SCAN_ANGULAROFFSETS_AFTER_H
#define MULTILAYER_LASER_SCAN_ANGULAROFFSETS_AFTER_H

namespace sensor_msgs
{
typedef multilayer_laser_scan::AngularOffsets AngularOffsets;
typedef multilayer_laser_scan::AngularOffsetsPtr AngularOffsetsPtr;
typedef multilayer_laser_scan::AngularOffsetsConstPtr AngularOffsetsConstPtr;
}

namespace multilayer_laser_scan
{

inline bool operator==(const AngularOffsets& lhs, const AngularOffsets& rhs)
{
  if (lhs.regular != rhs.regular)
    return false;

  if (lhs.regular)
  {
    if (lhs.min != rhs.min &&
        lhs.min != (rhs.min + 2 * M_PI) &&
        lhs.min != (rhs.min - 2 * M_PI))
      return false;

    if (lhs.max != rhs.max &&
        lhs.max != (rhs.max + 2 * M_PI) &&
        lhs.max != (rhs.max - 2 * M_PI))
      return false;

    if (lhs.increment == rhs.increment && lhs.increment != 0) return true;

    const auto lIncrement = (lhs.increment != 0) ? lhs.increment : rhs.increment;
    const auto rIncrement = (rhs.increment != 0) ? rhs.increment : lhs.increment;
    const auto lSamples = (lhs.samples != 0) ? lhs.samples : rhs.samples;
    const auto rSamples = (rhs.samples != 0) ? rhs.samples : lhs.samples;

    if (lIncrement != rIncrement) return false;
    if (lSamples != rSamples) return false;
  }
  else
  {
    if (lhs.offsets != rhs.offsets) return false;
  }

  return true;
}

inline bool operator!=(const AngularOffsets& lhs, const AngularOffsets& rhs)
{
  return !(lhs == rhs);
}

}

#endif //MULTILAYER_LASER_SCAN_ANGULAROFFSETS_AFTER_H
