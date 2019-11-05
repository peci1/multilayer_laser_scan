#ifndef MULTILAYER_LASER_SCAN_SCANLAYOUT_AFTER_H
#define MULTILAYER_LASER_SCAN_SCANLAYOUT_AFTER_H

namespace sensor_msgs
{
typedef multilayer_laser_scan::ScanLayout ScanLayout;
typedef multilayer_laser_scan::ScanLayoutPtr ScanLayoutPtr;
typedef multilayer_laser_scan::ScanLayoutConstPtr ScanLayoutConstPtr;
}

namespace multilayer_laser_scan
{

inline bool operator==(const ScanLayout& lhs, const ScanLayout& rhs)
{
  return lhs.angular_offsets == rhs.angular_offsets && lhs.time_offsets == rhs.time_offsets;
}

inline bool operator!=(const ScanLayout& lhs, const ScanLayout& rhs)
{
  return !(lhs == rhs);
}

}

#endif //MULTILAYER_LASER_SCAN_SCANLAYOUT_AFTER_H
