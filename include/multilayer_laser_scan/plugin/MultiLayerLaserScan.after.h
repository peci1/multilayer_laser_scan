#ifndef MULTILAYER_LASER_SCAN_MULTILAYERLASERSCAN_AFTER_H
#define MULTILAYER_LASER_SCAN_MULTILAYERLASERSCAN_AFTER_H

namespace sensor_msgs
{
typedef multilayer_laser_scan::MultiLayerLaserScan MultiLayerLaserScan;
typedef multilayer_laser_scan::MultiLayerLaserScanPtr MultiLayerLaserScanPtr;
typedef multilayer_laser_scan::MultiLayerLaserScanConstPtr MultiLayerLaserScanConstPtr;
}

namespace multilayer_laser_scan
{

inline bool operator==(const MultiLayerLaserScan& lhs, const MultiLayerLaserScan& rhs)
{
  if (lhs.header.frame_id != rhs.header.frame_id || lhs.header.stamp != rhs.header.stamp)
    return false;

  if (lhs.scan_layout != rhs.scan_layout)
    return false;
  if (lhs.subscan_layout != rhs.subscan_layout)
    return false;
  if (lhs.scan_offsets_during_subscan != rhs.scan_offsets_during_subscan)
    return false;

  if (lhs.range_min != rhs.range_min || lhs.range_max != rhs.range_max)
    return false;

  if (lhs.ranges != rhs.ranges || lhs.intensities != rhs.intensities)
    return false;

  if (lhs.custom_data != rhs.custom_data)
    return false;

  return true;
}

inline bool operator!=(const MultiLayerLaserScan& lhs, const MultiLayerLaserScan& rhs)
{
  return !(lhs == rhs);
}

}

#endif //MULTILAYER_LASER_SCAN_MULTILAYERLASERSCAN_AFTER_H
