#include "gtest/gtest.h"
#include <multilayer_laser_scan/scan_iterator.h>

using namespace sensor_msgs;

TEST(ScanIterator, Simple)
{
  MultiLayerLaserScan msg;

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1)};
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {0.0, 0.1};

  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0)};
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {0.0, 1.0};

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0.0, 0.0 };

  msg.ranges = {1, 2, 3, 4};
  msg.intensities = {5, 6, 7, 8};

  msg.header.stamp = ros::Time(10.0);

  auto layout = std::make_shared<MultiLayerLaserScanLayout>(msg);

  MultiLayerLaserScanBaseFieldsIterator it(msg, layout);
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(10.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(1, *(*it).range);
  EXPECT_DOUBLE_EQ(5, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(10.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(2, *(*it).range);
  EXPECT_DOUBLE_EQ(6, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(3, *(*it).range);
  EXPECT_DOUBLE_EQ(7, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(4, *(*it).range);
  EXPECT_DOUBLE_EQ(8, *(*it).intensity);

  *(*it).range = 10;
  EXPECT_EQ(10, msg.ranges.back());

  *(*it).intensity = 11;
  EXPECT_EQ(11, msg.intensities.back());

  ++it;
  EXPECT_FALSE(it.end() != it);
}

TEST(ScanIterator, EmptyIntensities)
{
  MultiLayerLaserScan msg;

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1)};
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {0.0, 0.1};

  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0)};
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {0.0, 1.0};

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0.0, 0.0 };

  msg.ranges = {1, 2, 3, 4};

  msg.header.stamp = ros::Time(10.0);

  auto layout = std::make_shared<MultiLayerLaserScanLayout>(msg);

  MultiLayerLaserScanBaseFieldsIterator it(msg, layout);
  EXPECT_TRUE(it.end() != it);

  auto data = *it;

  EXPECT_DOUBLE_EQ(ros::Duration(10.0).toSec(), data.timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, data.scanAngle);
  EXPECT_DOUBLE_EQ(0, data.subscanAngle);
  EXPECT_DOUBLE_EQ(1, *data.range);
  EXPECT_EQ(nullptr, data.intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(10.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(2, *(*it).range);
  EXPECT_EQ(nullptr, (*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(3, *(*it).range);
  EXPECT_EQ(nullptr, (*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(4, *(*it).range);
  EXPECT_EQ(nullptr, (*it).intensity);

  *(*it).range = 10;
  EXPECT_EQ(10, msg.ranges.back());

  ++it;
  EXPECT_FALSE(it.end() != it);
}

TEST(ScanIterator, ConstIterator)
{
  MultiLayerLaserScan msg;

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1)};
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {0.0, 0.1};

  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0)};
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {0.0, 1.0};

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0.0, 0.0 };

  msg.ranges = {1, 2, 3, 4};
  msg.intensities = {5, 6, 7, 8};

  msg.header.stamp = ros::Time(10.0);

  auto layout = std::make_shared<MultiLayerLaserScanLayout>(msg);

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, layout);
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(10.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(1, *(*it).range);
  EXPECT_DOUBLE_EQ(5, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(10.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(2, *(*it).range);
  EXPECT_DOUBLE_EQ(6, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(3, *(*it).range);
  EXPECT_DOUBLE_EQ(7, *(*it).intensity);

  ++it;
  EXPECT_TRUE(it.end() != it);

  EXPECT_DOUBLE_EQ(ros::Duration(11.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(4, *(*it).range);
  EXPECT_DOUBLE_EQ(8, *(*it).intensity);

  ++it;
  EXPECT_FALSE(it.end() != it);
}

TEST(ScanIterator, CustomDataConstIterator)
{
  MultiLayerLaserScan msg;

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1)};
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {0.0, 0.1};

  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0)};
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {0.0, 1.0};

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0.0, 0.0 };

  msg.ranges = {1, 2, 3, 4};
  msg.intensities = {5, 6, 7, 8};

  PointField field;
  field.count = 1;
  field.name = "ring";
  field.datatype = PointField::UINT16;
  field.offset = 0;

  msg.custom_data.fields.push_back(field);
  msg.custom_data.point_step = 2;
  msg.custom_data.is_bigendian = false;
  msg.custom_data.data.resize(8);
  auto rings = reinterpret_cast<uint16_t*>(&msg.custom_data.data.front());
  rings[0] = 20;
  rings[1] = 30;
  rings[2] = 40;
  rings[3] = 50;

  msg.header.stamp = ros::Time(10.0);

  auto layout = std::make_shared<MultiLayerLaserScanLayout>(msg);

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, layout);
  PointDataConstIterator<uint16_t> rit(msg.custom_data, "ring");

  EXPECT_TRUE(it.end() != it);
  EXPECT_TRUE(rit.end() != rit);

  EXPECT_DOUBLE_EQ(ros::Duration(10.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(1, *(*it).range);
  EXPECT_DOUBLE_EQ(5, *(*it).intensity);
  EXPECT_DOUBLE_EQ(20, *rit);

  ++it; ++rit;
  EXPECT_TRUE(it.end() != it);
  EXPECT_TRUE(rit.end() != rit);

  EXPECT_DOUBLE_EQ(ros::Duration(10.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(2, *(*it).range);
  EXPECT_DOUBLE_EQ(6, *(*it).intensity);
  EXPECT_DOUBLE_EQ(30, *rit);

  ++it; ++rit;
  EXPECT_TRUE(it.end() != it);
  EXPECT_TRUE(rit.end() != rit);

  EXPECT_DOUBLE_EQ(ros::Duration(11.0).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(3, *(*it).range);
  EXPECT_DOUBLE_EQ(7, *(*it).intensity);
  EXPECT_DOUBLE_EQ(40, *rit);

  ++it; ++rit;
  EXPECT_TRUE(it.end() != it);
  EXPECT_TRUE(rit.end() != rit);

  EXPECT_DOUBLE_EQ(ros::Duration(11.1).toSec(), (*it).timestamp.toSec());
  EXPECT_DOUBLE_EQ(1.0, (*it).scanAngle);
  EXPECT_DOUBLE_EQ(0.1, (*it).subscanAngle);
  EXPECT_DOUBLE_EQ(4, *(*it).range);
  EXPECT_DOUBLE_EQ(8, *(*it).intensity);
  EXPECT_DOUBLE_EQ(50, *rit);

  ++it; ++rit;
  EXPECT_FALSE(it.end() != it);
  EXPECT_FALSE(rit.end() != rit);
}

TEST(ScanIterator, ConstructPointData)
{
  PointData msg;
  PointDataModifier mod(msg);

  mod.setFields(3,
      "ring", 1, PointField::UINT16,
      "normal", 3, PointField::FLOAT32,
      "rgba", 1, PointField::FLOAT32);

  ASSERT_EQ(3, msg.fields.size());

  EXPECT_EQ(0, msg.fields[0].offset);
  EXPECT_EQ(PointField::UINT16, msg.fields[0].datatype);
  EXPECT_EQ(1, msg.fields[0].count);
  EXPECT_EQ("ring", msg.fields[0].name);

  EXPECT_EQ(2, msg.fields[1].offset);
  EXPECT_EQ(PointField::FLOAT32, msg.fields[1].datatype);
  EXPECT_EQ(3, msg.fields[1].count);
  EXPECT_EQ("normal", msg.fields[1].name);

  EXPECT_EQ(14, msg.fields[2].offset);
  EXPECT_EQ(PointField::FLOAT32, msg.fields[2].datatype);
  EXPECT_EQ(1, msg.fields[2].count);
  EXPECT_EQ("rgba", msg.fields[2].name);

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(0, msg.data.size());

  mod.resize(5);

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(90, msg.data.size());
  EXPECT_EQ(5, mod.size());

  mod.clear();

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(0, msg.data.size());
  EXPECT_EQ(0, mod.size());

  mod.setFieldsByString(2, "ring", "normal");
  ASSERT_EQ(2, msg.fields.size());

}

TEST(ScanIterator, ConstructPointDataFromString)
{
  PointData msg;
  PointDataModifier mod(msg);

  mod.setFieldsByString(3, "ring", "normal", "rgba");

  ASSERT_EQ(3, msg.fields.size());

  EXPECT_EQ(0, msg.fields[0].offset);
  EXPECT_EQ(PointField::UINT16, msg.fields[0].datatype);
  EXPECT_EQ(1, msg.fields[0].count);
  EXPECT_EQ("ring", msg.fields[0].name);

  EXPECT_EQ(2, msg.fields[1].offset);
  EXPECT_EQ(PointField::FLOAT32, msg.fields[1].datatype);
  EXPECT_EQ(3, msg.fields[1].count);
  EXPECT_EQ("normal", msg.fields[1].name);

  EXPECT_EQ(14, msg.fields[2].offset);
  EXPECT_EQ(PointField::FLOAT32, msg.fields[2].datatype);
  EXPECT_EQ(1, msg.fields[2].count);
  EXPECT_EQ("rgba", msg.fields[2].name);

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(0, msg.data.size());

  mod.resize(5);

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(90, msg.data.size());
  EXPECT_EQ(5, mod.size());

  mod.clear();

  EXPECT_EQ(18, msg.point_step);
  EXPECT_EQ(0, msg.data.size());
  EXPECT_EQ(0, mod.size());

  mod.setFieldsByString(2, "ring", "normal");
  ASSERT_EQ(2, msg.fields.size());
}

TEST(ScanIterator, FillDataByIterators)
{
  MultiLayerLaserScan msg;
  msg.header.stamp = ros::Time(10.0);

  PointDataModifier mod(msg.custom_data);
  mod.setFieldsByString(3, "ring", "normal", "rgba");
  mod.resize(4);

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1)};
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {0.0, 0.1};

  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0)};
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {0.0, 1.0};

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0.0, 0.0 };

  msg.ranges.resize(4, 0.0);
  msg.intensities.resize(4, 0.0);

  MultiLayerLaserScanBaseFieldsIterator it(msg, std::make_shared<MultiLayerLaserScanLayout>(msg));
  PointDataIterator<uint16_t> rit(msg.custom_data, "ring");
  PointDataIterator<float> nit(msg.custom_data, "normal");
  PointDataIterator<uint8_t> cit(msg.custom_data, "rgba");

  for (size_t i = 0; it != it.end(); ++it, ++rit, ++nit, ++cit, ++i)
  {
    *(*it).range = i * 2.0;
    *(*it).intensity = i * 3.0;
    *rit = i * 4;
    nit[0] = i * 5.0;
    nit[1] = i * 6.0;
    nit[2] = i * 7.0;
    cit[0] = i * 8;
    cit[1] = i * 9;
    cit[2] = i * 10;
  }

  ASSERT_EQ(3, msg.custom_data.fields.size());
  ASSERT_EQ(4, msg.ranges.size());
  ASSERT_EQ(4, msg.intensities.size());
  ASSERT_EQ(72, msg.custom_data.data.size());

  ASSERT_EQ(0.0, msg.ranges[0]);
  ASSERT_EQ(2.0, msg.ranges[1]);
  ASSERT_EQ(4.0, msg.ranges[2]);
  ASSERT_EQ(6.0, msg.ranges[3]);

  ASSERT_EQ(0.0, msg.intensities[0]);
  ASSERT_EQ(3.0, msg.intensities[1]);
  ASSERT_EQ(6.0, msg.intensities[2]);
  ASSERT_EQ(9.0, msg.intensities[3]);

  const auto step = msg.custom_data.point_step;

  // ring
  EXPECT_EQ(0,  reinterpret_cast<uint16_t*>(&msg.custom_data.data[0 * step])[0]);
  EXPECT_EQ(4,  reinterpret_cast<uint16_t*>(&msg.custom_data.data[1 * step])[0]);
  EXPECT_EQ(8,  reinterpret_cast<uint16_t*>(&msg.custom_data.data[2 * step])[0]);
  EXPECT_EQ(12, reinterpret_cast<uint16_t*>(&msg.custom_data.data[3 * step])[0]);

  // normal.x
  EXPECT_DOUBLE_EQ(0.0,  reinterpret_cast<float*>(&msg.custom_data.data[0 * step + 2])[0]);
  EXPECT_DOUBLE_EQ(5.0,  reinterpret_cast<float*>(&msg.custom_data.data[1 * step + 2])[0]);
  EXPECT_DOUBLE_EQ(10.0, reinterpret_cast<float*>(&msg.custom_data.data[2 * step + 2])[0]);
  EXPECT_DOUBLE_EQ(15.0, reinterpret_cast<float*>(&msg.custom_data.data[3 * step + 2])[0]);
  // normal.y
  EXPECT_DOUBLE_EQ(0.0,  reinterpret_cast<float*>(&msg.custom_data.data[0 * step + 2])[1]);
  EXPECT_DOUBLE_EQ(6.0,  reinterpret_cast<float*>(&msg.custom_data.data[1 * step + 2])[1]);
  EXPECT_DOUBLE_EQ(12.0, reinterpret_cast<float*>(&msg.custom_data.data[2 * step + 2])[1]);
  EXPECT_DOUBLE_EQ(18.0, reinterpret_cast<float*>(&msg.custom_data.data[3 * step + 2])[1]);
  // normal.z
  EXPECT_DOUBLE_EQ(0.0,  reinterpret_cast<float*>(&msg.custom_data.data[0 * step + 2])[2]);
  EXPECT_DOUBLE_EQ(7.0,  reinterpret_cast<float*>(&msg.custom_data.data[1 * step + 2])[2]);
  EXPECT_DOUBLE_EQ(14.0, reinterpret_cast<float*>(&msg.custom_data.data[2 * step + 2])[2]);
  EXPECT_DOUBLE_EQ(21.0, reinterpret_cast<float*>(&msg.custom_data.data[3 * step + 2])[2]);

  // color.r
  EXPECT_EQ(0,  reinterpret_cast<uint8_t*>(&msg.custom_data.data[0 * step + 14])[0]);
  EXPECT_EQ(8,  reinterpret_cast<uint8_t*>(&msg.custom_data.data[1 * step + 14])[0]);
  EXPECT_EQ(16, reinterpret_cast<uint8_t*>(&msg.custom_data.data[2 * step + 14])[0]);
  EXPECT_EQ(24, reinterpret_cast<uint8_t*>(&msg.custom_data.data[3 * step + 14])[0]);
  // color.g
  EXPECT_EQ(0,  reinterpret_cast<uint8_t*>(&msg.custom_data.data[0 * step + 14])[1]);
  EXPECT_EQ(9,  reinterpret_cast<uint8_t*>(&msg.custom_data.data[1 * step + 14])[1]);
  EXPECT_EQ(18, reinterpret_cast<uint8_t*>(&msg.custom_data.data[2 * step + 14])[1]);
  EXPECT_EQ(27, reinterpret_cast<uint8_t*>(&msg.custom_data.data[3 * step + 14])[1]);
  // color.b
  EXPECT_EQ(0,  reinterpret_cast<uint8_t*>(&msg.custom_data.data[0 * step + 14])[2]);
  EXPECT_EQ(10, reinterpret_cast<uint8_t*>(&msg.custom_data.data[1 * step + 14])[2]);
  EXPECT_EQ(20, reinterpret_cast<uint8_t*>(&msg.custom_data.data[2 * step + 14])[2]);
  EXPECT_EQ(30, reinterpret_cast<uint8_t*>(&msg.custom_data.data[3 * step + 14])[2]);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}