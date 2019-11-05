#ifndef MULTILAYER_LASER_SCAN_POINTDATA_AFTER_H
#define MULTILAYER_LASER_SCAN_POINTDATA_AFTER_H

namespace sensor_msgs
{
typedef multilayer_laser_scan::PointData PointData;
typedef multilayer_laser_scan::PointDataPtr PointDataPtr;
typedef multilayer_laser_scan::PointDataConstPtr PointDataConstPtr;
}

namespace multilayer_laser_scan
{

inline bool operator==(const PointData& lhs, const PointData& rhs)
{
  if (lhs.point_step != rhs.point_step)
    return false;

  if (lhs.fields.size() != rhs.fields.size())
    return false;

  for (size_t i = 0; i < lhs.fields.size(); ++i)
  {
    if (lhs.fields[i].datatype != rhs.fields[i].datatype) return false;
    if (lhs.fields[i].name != rhs.fields[i].name) return false;
    if (lhs.fields[i].count != rhs.fields[i].count) return false;
    if (lhs.fields[i].offset != rhs.fields[i].offset) return false;
  }

  if (lhs.data != rhs.data)
    return false;

  return true;
}

inline bool operator!=(const PointData& lhs, const PointData& rhs)
{
  return !(lhs == rhs);
}

}

#endif //MULTILAYER_LASER_SCAN_POINTDATA_AFTER_H
