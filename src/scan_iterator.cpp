#include <multilayer_laser_scan/scan_iterator.h>

namespace sensor_msgs
{

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFields<C, R, I>::MultiLayerLaserScanBaseFields(
    const double _subscanAngle, const double _scanAngle,
    R* _range, I* _intensity, const ros::Time _timestamp)
    : subscanAngle(_subscanAngle), scanAngle(_scanAngle), range(_range),
      intensity(_intensity), timestamp(_timestamp)
{
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::MultiLayerLaserScanBaseFieldsIteratorBase(
    C &scan, std::shared_ptr<MultiLayerLaserScanLayout> layout)
    : scan(&scan), layout(std::move(layout))
{
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>&
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::operator=(
    const MultiLayerLaserScanBaseFieldsIteratorBase &iter)
{
  if (this != &iter)
  {
    this->scan = iter.scan;
    this->layout = iter.layout;
    this->i = iter.i;
  }
  return *this;
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFields<C, R, I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::operator*() const
{
  double scanAngle = std::numeric_limits<double>::quiet_NaN();
  double subscanAngle = std::numeric_limits<double>::quiet_NaN();
  ros::Duration timeOffset;

  this->layout->GetAll(this->i, scanAngle, subscanAngle, timeOffset);

  ros::Time timestamp = this->scan->header.stamp + timeOffset;

  return MultiLayerLaserScanBaseFields<C, R, I>(subscanAngle, scanAngle,
      &this->scan->ranges[i],
      this->scan->intensities.empty() ? nullptr : &this->scan->intensities[i],
      timestamp);
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>&
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::operator++()
{
  ++this->i;
  return *this;
}

template<typename C, typename R, typename I>
bool MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::operator!=(
    const MultiLayerLaserScanBaseFieldsIteratorBase& iter) const
{
  return this->i != iter.i;
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::end() const
{
  MultiLayerLaserScanBaseFieldsIteratorBase result(*this->scan, this->layout);
  result.i = this->layout->Length();
  return result;
}

template<typename C, typename R, typename I>
MultiLayerLaserScanBaseFieldsIteratorBase<C, R, I>::~MultiLayerLaserScanBaseFieldsIteratorBase() = default;

PointDataModifier::PointDataModifier(PointData& pointDataMsg) :
  pointDataMsg(pointDataMsg)
{
}

size_t PointDataModifier::size() const
{
  return pointDataMsg.data.size() / pointDataMsg.point_step;
}

void PointDataModifier::reserve(size_t size)
{
  pointDataMsg.data.reserve(size * pointDataMsg.point_step);
}

void PointDataModifier::resize(size_t size)
{
  pointDataMsg.data.resize(size * pointDataMsg.point_step);
}

void PointDataModifier::clear()
{
  pointDataMsg.data.clear();
}

/** Return the size of a datatype (which is an enum of sensor_msgs::PointField::) in bytes
 * @param datatype one of the enums of sensor_msgs::PointField::
 */
inline size_t sizeOfPointField(PointField::_datatype_type datatype)
{
  if ((datatype == PointField::INT8) || (datatype == PointField::UINT8))
    return 1;
  else if ((datatype == PointField::INT16) || (datatype == PointField::UINT16))
    return 2;
  else if ((datatype == PointField::INT32) || (datatype == PointField::UINT32) ||
      (datatype == PointField::FLOAT32))
    return 4;
  else if (datatype == PointField::FLOAT64)
    return 8;
  else
  {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
}

/** Private function that adds a PointField to the "fields" member of a PointCloud2
 * @param pointDataMsg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointField that will be added to the PointCLoud2
 */
inline size_t addPointField(sensor_msgs::PointData &pointDataMsg,
    const std::string &name, PointField::_count_type count,
    PointField::_datatype_type datatype, PointField::_offset_type offset)
{
  PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  pointDataMsg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * sizeOfPointField(datatype);
}

void PointDataModifier::setFields(size_t n_fields, ...)
{
  pointDataMsg.fields.clear();
  pointDataMsg.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  size_t offset = 0;
  for (size_t i = 0; i < n_fields; ++i) {
    // Create the corresponding PointField
    const auto name = static_cast<std::string>(va_arg(vl, char*));
    const auto count = static_cast<PointField::_count_type>(va_arg(vl, int));
    const auto datatype = static_cast<PointField::_datatype_type>(va_arg(vl, int));
    offset = addPointField(pointDataMsg, name, count, datatype,
        static_cast<PointField::_offset_type >(offset));
  }
  va_end(vl);

  size_t numPoints = 0;
  if (pointDataMsg.point_step > 0)
    numPoints = pointDataMsg.data.size() / pointDataMsg.point_step;

  // Resize the point cloud accordingly
  pointDataMsg.point_step = offset;
  this->resize(numPoints);
}

void PointDataModifier::setFieldsByString(size_t n_fields, ...)
{
  pointDataMsg.fields.clear();
  pointDataMsg.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  size_t offset = 0;
  for (size_t i = 0; i < n_fields; ++i) {
    // Create the corresponding PointFields
    const auto fieldName = static_cast<std::string>(va_arg(vl, char*));
    const auto fieldAdded = this->addPointFieldByString(fieldName, offset);
    if (!fieldAdded) {
      va_end(vl);
      throw std::runtime_error("Field " + fieldName + " does not exist");
    }
  }
  va_end(vl);

  size_t numPoints = 0;
  if (pointDataMsg.point_step > 0)
    numPoints = pointDataMsg.data.size() / pointDataMsg.point_step;

  // Resize the point cloud accordingly
  pointDataMsg.point_step = offset;
  this->resize(numPoints);
}

bool PointDataModifier::addPointFieldByString(const std::string &fieldName, size_t& offset)
{
  if (fieldName == "rgb" || fieldName == "rgba" || fieldName == "strongest" ||
      fieldName == "latest" || fieldName == "reflectivity" || fieldName == "noise")
  {
    offset = addPointField(pointDataMsg, fieldName, 1, sensor_msgs::PointField::FLOAT32, offset);
    return true;
  }
  else if (fieldName == "ring")
  {
    offset = addPointField(pointDataMsg, fieldName, 1, sensor_msgs::PointField::UINT16, offset);
    return true;
  }
  else if (fieldName == "normal")
  {
    offset = addPointField(pointDataMsg, fieldName, 3, sensor_msgs::PointField::FLOAT32, offset);
    return true;
  }
  return false;
}

}
