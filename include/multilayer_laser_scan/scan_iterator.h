#ifndef MULTILAYER_LASER_SCAN_SCAN_ITERATOR_H
#define MULTILAYER_LASER_SCAN_SCAN_ITERATOR_H

// HACK
#include <sstream>
#define private protected
#include <sensor_msgs/point_cloud2_iterator.h>
#undef private
// HACK

#include <multilayer_laser_scan/MultiLayerLaserScan.h>
#include <multilayer_laser_scan/MultiLayerLaserScanLayout.h>

#include <memory>

namespace sensor_msgs
{

template<typename C, typename R, typename I>
struct MultiLayerLaserScanBaseFields
{
  MultiLayerLaserScanBaseFields(double _subscanAngle, double _scanAngle,
    R* _range, I* _intensity, ros::Time _timestamp);

  double subscanAngle = 0.0;
  double scanAngle = 0.0;
  R* range = nullptr;
  I* intensity = nullptr;
  ros::Time timestamp;
};

template<typename C, typename R, typename I>
class MultiLayerLaserScanBaseFieldsIteratorBase
{
  public: MultiLayerLaserScanBaseFieldsIteratorBase(
      C& scan, std::shared_ptr<MultiLayerLaserScanLayout> layout);

  virtual ~MultiLayerLaserScanBaseFieldsIteratorBase();

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  MultiLayerLaserScanBaseFieldsIteratorBase& operator =(const MultiLayerLaserScanBaseFieldsIteratorBase& iter);

  /** Dereference the iterator.
   * @return the value to which the iterator is pointing
   */
  MultiLayerLaserScanBaseFields<C, R, I> operator *() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  MultiLayerLaserScanBaseFieldsIteratorBase& operator ++();

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator !=(const MultiLayerLaserScanBaseFieldsIteratorBase& iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  MultiLayerLaserScanBaseFieldsIteratorBase end() const;

  protected: C* scan;
  protected: std::shared_ptr<MultiLayerLaserScanLayout> layout;
  protected: size_t i = 0;
};

typedef MultiLayerLaserScanBaseFieldsIteratorBase<MultiLayerLaserScan, float, float> MultiLayerLaserScanBaseFieldsIterator;
typedef MultiLayerLaserScanBaseFieldsIteratorBase<const MultiLayerLaserScan, const float, const float> MultiLayerLaserScanBaseFieldsConstIterator;

// instantiate the templates
template class MultiLayerLaserScanBaseFieldsIteratorBase<MultiLayerLaserScan, float, float>;
template class MultiLayerLaserScanBaseFieldsIteratorBase<const MultiLayerLaserScan, const float, const float>;

namespace impl
{

template<typename T, typename TT, typename U, typename C, template<typename> class V>
class PointDataIteratorBase : public PointCloud2IteratorBase<T, TT, U, C, V>
{
  public: PointDataIteratorBase(C &pointDataMsg, const std::string &fieldName)
  {
    int offset = set_field(pointDataMsg, fieldName);

    this->data_char_ = &(pointDataMsg.data.front()) + offset;
    this->data_ = reinterpret_cast<TT *>(this->data_char_);
    this->data_end_ = reinterpret_cast<TT *>(&(pointDataMsg.data.back()) + 1 + offset);
  }

  /** Common code to set the field of the PointData
  * @param pointDataMsg the PointData to modify
  * @param fieldName the name of the field to iterate upon
  * @return the offset at which the field is found
  */
  size_t set_field(const sensor_msgs::PointData &pointDataMsg, const std::string &fieldName)
  {
    this->is_bigendian_ = pointDataMsg.is_bigendian;
    this->point_step_ = pointDataMsg.point_step;
    // make sure the channel is valid
    auto field_iter = pointDataMsg.fields.begin();
    auto field_end = pointDataMsg.fields.end();
    while ((field_iter != field_end) && (field_iter->name != fieldName))
      ++field_iter;

    if (field_iter == field_end)
    {
      // Handle the special case of r,g,b,a (we assume they are understood as the channels of an rgb or rgba field)
      if ((fieldName == "r") || (fieldName == "g") || (fieldName == "b") || (fieldName == "a"))
      {
        // Check that rgb or rgba is present
        field_iter = pointDataMsg.fields.begin();
        while ((field_iter != field_end) && (field_iter->name != "rgb")
            && (field_iter->name != "rgba"))
          ++field_iter;
        if (field_iter == field_end)
          throw std::runtime_error("Field " + fieldName + " does not exist");
        if (fieldName == "r")
        {
          if (this->is_bigendian_)
            return field_iter->offset + 1;
          else
            return field_iter->offset + 2;
        }
        if (fieldName == "g")
        {
          if (this->is_bigendian_)
            return field_iter->offset + 2;
          else
            return field_iter->offset + 1;
        }
        if (fieldName == "b")
        {
          if (this->is_bigendian_)
            return field_iter->offset + 3;
          else
            return field_iter->offset + 0;
        }
        if (fieldName == "a")
        {
          if (this->is_bigendian_)
            return field_iter->offset + 0;
          else
            return field_iter->offset + 3;
        }
      }
      else
        throw std::runtime_error("Field " + fieldName + " does not exist");
    }

    return field_iter->offset;
  }
};

}

/**
 * \brief Class that can iterate over PointData
 *
 * T type of the element being iterated upon
 * E.g, you create your PointData message as follows:
 * <PRE>
 *   modifier.setFieldsByString(pointDataMsg, 2, "strongest", "ring");
 * </PRE>
 *
 * For iterating over ring, you do :
 * <PRE>
 *   sensor_msgs::PointDataIterator<uint16_t> iter_ring(pointDataMsg, "ring");
 * </PRE>
 * and then access ring through iter_ring[0] or *iter_ring.
 *
 * For iterating over RGB, you do:
 * <PRE>
 * sensor_msgs::PointDataIterator<uint8_t> iter_rgb(pointDataMsg, "rgb");
 * </PRE>
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
template<typename T>
class PointDataIterator :
    public impl::PointDataIteratorBase<T, T, unsigned char, PointData, PointDataIterator>
{
  public: PointDataIterator(sensor_msgs::PointData& pointData, const std::string &fieldName) :
    impl::PointDataIteratorBase<T, T, unsigned char, PointData, PointDataIterator>::
      PointDataIteratorBase(pointData, fieldName) {}
};

/**
 * \brief Same as a PointDataIterator but for const data
 */
template<typename T>
class PointDataConstIterator :
    public impl::PointDataIteratorBase<
        T, const T, const unsigned char, const PointData, PointDataConstIterator>
{
  public: PointDataConstIterator(
      const sensor_msgs::PointData& pointData, const std::string &fieldName) :
    impl::PointDataIteratorBase<
        T, const T, const unsigned char, const PointData, PointDataConstIterator>::
      PointDataIteratorBase(pointData, fieldName) {}
};

/**
 * @brief Enables modifying a sensor_msgs::PointData like a container
 */
class PointDataModifier
{
  /**
   * @brief Default constructor
   * @param cloud_msg The sensor_msgs::PointData to modify
   */
  public: explicit PointDataModifier(PointData& pointDataMsg);
  public: virtual ~PointDataModifier() = default;

  /**
   * @return the number of T's in the original sensor_msgs::PointData
   */
  public: size_t size() const;

  /**
   * @param size The number of T's to reserve in the original sensor_msgs::PointData for
   */
  public: void reserve(size_t size);

  /**
   * @param size The number of T's to change the size of the original sensor_msgs::PointData by
   */
  public: void resize(size_t size);

  /**
   * @brief remove all T's from the original sensor_msgs::PointData
   */
  public: void clear();

  /**
   * @brief Function setting some fields in PointData and adjusting the
   *        internals of the PointData
   * @param n_fields the number of fields to add. The fields are given as
   *        triplets: name of the field as char*, number of elements in the
   *        field, the datatype of the elements in the field
   *
   * E.g, you create your PointData message with XYZ/RGB as follows:
   * <PRE>
   *   setFields(4, "strongest", 1, sensor_msgs::PointField::FLOAT32,
  *                 "ring", 1, sensor_msgs::PointField::UINT16,
  *                 "rgb", 1, sensor_msgs::PointField::FLOAT32);
   * </PRE>
   * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING
   * For simple usual cases, the overloaded setFieldsByString is what you want.
   */
  public: void setFields(size_t n_fields, ...);

  /**
   * @brief Function setting some fields in a PointData and adjusting the
   *        internals of the PointData
   * @param n_fields the number of fields to add. The fields are given as
   *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
   *        "rgba" (4 uchar stacked in a float)
   * @return void
   *
   * WARNING: THIS FUNCTION DOES NOT ADD ANY PADDING
   */
  public: void setFieldsByString(size_t n_fields, ...);

  protected: virtual bool addPointFieldByString(const std::string& fieldName,
      size_t& offset);

  /** A reference to the original sensor_msgs::PointData that we read */
  protected: PointData& pointDataMsg;
};

}

#endif //MULTILAYER_LASER_SCAN_SCAN_ITERATOR_H
