#include "gtest/gtest.h"
#include <multilayer_laser_scan/MultiLayerLaserScanLayout.h>
#include <multilayer_laser_scan/scan_iterator.h>
#include <fstream>

#include <boost/algorithm/string.hpp>

using namespace sensor_msgs;

TEST(ScanLayout, TestRegularAngularOffsetsByIncrement)
{
  AngularOffsets msg, tmpMsg;
  msg.regular = true;
  msg.min = -1.0;
  msg.max = 1.0;
  msg.increment = 1.0;

  EXPECT_THROW((ExplicitAngularOffsets(msg)), std::runtime_error);

  RegularAngularOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // reversed direction

  msg.increment = -1;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // 360 deg scan with end point excluded

  msg.min = -M_PI;
  msg.max = M_PI;
  msg.increment = M_PI;
  msg.exclude_last = true;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_THROW(parsed.Get(2), std::out_of_range);

  // 360 deg scan with end point included

  msg.min = -M_PI;
  msg.max = M_PI;
  msg.increment = M_PI;
  msg.exclude_last = false;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(M_PI, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // less than 360 deg scan with end point excluded

  msg.min = -M_PI;
  msg.max = 0;
  msg.increment = M_PI_2;
  msg.exclude_last = true;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI_2, parsed.Get(1));
  EXPECT_THROW(parsed.Get(2), std::out_of_range);

  // single-element construction succeeds

  msg.min = -M_PI;
  msg.max = -M_PI;
  msg.increment = M_PI;
  msg.exclude_last = false;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  // single-element construction with large increment succeeds

  msg.min = -M_PI;
  msg.max = 0;
  msg.increment = 2 * M_PI;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  // swapped min/max isn't allowed

  msg.min = M_PI;
  msg.max = -M_PI;

  EXPECT_THROW((RegularAngularOffsets(msg)), std::runtime_error);

  // if the range is not divisible by increment, do your best

  msg.min = -M_PI;
  msg.max = M_PI_2;
  msg.increment = 1;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(5, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI + 1, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-M_PI + 2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-M_PI + 3, parsed.Get(3));
  EXPECT_DOUBLE_EQ(-M_PI + 4, parsed.Get(4));
  EXPECT_THROW(parsed.Get(5), std::out_of_range);

  msg.increment = -1;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(5, parsed.Length());
  EXPECT_DOUBLE_EQ(M_PI_2, parsed.Get(0));
  EXPECT_DOUBLE_EQ(M_PI_2 - 1, parsed.Get(1));
  EXPECT_DOUBLE_EQ(M_PI_2 - 2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(M_PI_2 - 3, parsed.Get(3));
  EXPECT_DOUBLE_EQ(M_PI_2 - 4, parsed.Get(4));
  EXPECT_THROW(parsed.Get(5), std::out_of_range);

  msg.min = -M_PI;
  msg.max = M_PI;
  msg.increment = 1;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(7, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI + 1, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-M_PI + 2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-M_PI + 3, parsed.Get(3));
  EXPECT_DOUBLE_EQ(-M_PI + 4, parsed.Get(4));
  EXPECT_DOUBLE_EQ(-M_PI + 5, parsed.Get(5));
  EXPECT_DOUBLE_EQ(-M_PI + 6, parsed.Get(6));
  EXPECT_THROW(parsed.Get(7), std::out_of_range);

  EXPECT_THROW(parsed.AddOffset(-M_PI + 8), std::runtime_error);
  EXPECT_THROW(parsed.AddOffset(-M_PI + 10), std::runtime_error);
  EXPECT_THROW(parsed.AddOffset(-M_PI + 7.1), std::runtime_error);

  parsed.AddOffset(-M_PI + 7);
  parsed.FillMsg(tmpMsg); msg.max = -M_PI + 7; EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(8, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI + 1, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-M_PI + 2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-M_PI + 3, parsed.Get(3));
  EXPECT_DOUBLE_EQ(-M_PI + 4, parsed.Get(4));
  EXPECT_DOUBLE_EQ(-M_PI + 5, parsed.Get(5));
  EXPECT_DOUBLE_EQ(-M_PI + 6, parsed.Get(6));
  EXPECT_DOUBLE_EQ(-M_PI + 7, parsed.Get(7));
  EXPECT_THROW(parsed.Get(8), std::out_of_range);

  parsed.AddOffset(-M_PI - 1);
  parsed.FillMsg(tmpMsg); msg.min = -M_PI - 1; EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(9, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI - 1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-M_PI + 1, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-M_PI + 2, parsed.Get(3));
  EXPECT_DOUBLE_EQ(-M_PI + 3, parsed.Get(4));
  EXPECT_DOUBLE_EQ(-M_PI + 4, parsed.Get(5));
  EXPECT_DOUBLE_EQ(-M_PI + 5, parsed.Get(6));
  EXPECT_DOUBLE_EQ(-M_PI + 6, parsed.Get(7));
  EXPECT_DOUBLE_EQ(-M_PI + 7, parsed.Get(8));
  EXPECT_THROW(parsed.Get(9), std::out_of_range);

  ASSERT_THROW(parsed.AddOffset(-M_PI - 1), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 0), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 1), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 2), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 3), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 4), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 5), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 6), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(-M_PI + 7), std::runtime_error);
}

TEST(ScanLayout, TestRegularAngularOffsetsBySamples)
{
  AngularOffsets msg, tmpMsg;
  msg.regular = true;
  msg.min = -1.0;
  msg.max = 1.0;
  msg.samples = 3;

  EXPECT_THROW((ExplicitAngularOffsets(msg)), std::runtime_error);

  RegularAngularOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // reversed direction

  msg.samples = -3;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // 360 deg scan with end point excluded

  msg.min = -M_PI;
  msg.max = M_PI;
  msg.samples = 2;
  msg.exclude_last = true;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_THROW(parsed.Get(2), std::out_of_range);

  // 360 deg scan with end point included

  msg.min = -M_PI;
  msg.max = M_PI;
  msg.samples = 3;
  msg.exclude_last = false;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(M_PI, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // less than 360 deg scan with end point excluded

  msg.min = -M_PI;
  msg.max = 0;
  msg.samples = 2;
  msg.exclude_last = true;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-M_PI_2, parsed.Get(1));
  EXPECT_THROW(parsed.Get(2), std::out_of_range);

  // single-element construction succeeds

  msg.min = -M_PI;
  msg.max = -M_PI;
  msg.samples = 1;
  msg.exclude_last = false;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  // single-element construction with nonempty range succeeds

  msg.min = -M_PI;
  msg.max = 0;
  msg.samples = 1;
  parsed = RegularAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);
}

TEST(ScanLayout, TestRegularAngularOffsetsBothIncrementAndSamples)
{
  AngularOffsets msg, tmpMsg;
  msg.regular = true;
  msg.min = -1.0;
  msg.max = 1.0;
  msg.samples = 3;
  msg.increment = 1.0;

  RegularAngularOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // reversed direction

  msg.increment = -1;
  msg.samples = -3;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  // increment has precedence on collision

  msg.increment = -1;
  msg.samples = 3;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  msg.increment = 1;
  msg.samples = 2;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);

  msg.increment = 1;
  msg.samples = 1;
  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(0, parsed.Get(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);
}

TEST(ScanLayout, TestRegularAngularOffsetsDegenerated)
{
  AngularOffsets msg, tmpMsg;
  msg.regular = true;
  msg.min = -1.0;
  msg.max = -1.0;

  msg.samples = 0;
  msg.increment = 1.0;

  RegularAngularOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = 1;
  msg.increment = 1.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = 0;
  msg.increment = 10.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = 1;
  msg.increment = -10.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = 1;
  msg.increment = 0.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = -1;
  msg.increment = 0.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  msg.samples = 0;
  msg.increment = 0.0;

  EXPECT_THROW((RegularAngularOffsets(msg)), std::runtime_error);

  // here we expect that the user wants to specify 10 zeros
  msg.samples = 10;
  msg.increment = 0.0;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(10, parsed.Length());
  for (size_t i = 0; i < 10; ++i)
    EXPECT_DOUBLE_EQ(-1, parsed.Get(i));
  EXPECT_THROW(parsed.Get(10), std::out_of_range);

  // here we expect that the user wanted to specify full circle
  msg.samples = 4;
  msg.increment = M_PI_2;
  msg.exclude_last = true;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(4, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-1 + 1 * M_PI_2, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1 + 2 * M_PI_2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-1 + 3 * M_PI_2, parsed.Get(3));
  EXPECT_THROW(parsed.Get(4), std::out_of_range);

  // here we expect that the user wanted to specify full circle, and that increment takes priority
  msg.samples = 5;
  msg.increment = M_PI_2;

  parsed = RegularAngularOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(4, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(-1 + 1 * M_PI_2, parsed.Get(1));
  EXPECT_DOUBLE_EQ(-1 + 2 * M_PI_2, parsed.Get(2));
  EXPECT_DOUBLE_EQ(-1 + 3 * M_PI_2, parsed.Get(3));
  EXPECT_THROW(parsed.Get(4), std::out_of_range);
}

TEST(ScanLayout, TestExplicitAngularOffsets)
{
  AngularOffsets msg, tmpMsg;
  msg.regular = false;
  msg.offsets = {-1.0, 1.0, 0.0, M_PI};

  EXPECT_THROW((RegularAngularOffsets(msg)), std::runtime_error);

  ExplicitAngularOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(4, parsed.Length());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(1, parsed.Get(1));
  EXPECT_DOUBLE_EQ(0, parsed.Get(2));
  EXPECT_DOUBLE_EQ(M_PI, parsed.Get(3));
  EXPECT_THROW(parsed.Get(4), std::out_of_range);

  // check empty construction doesn't succeed

  msg.offsets.clear();

  EXPECT_THROW((ExplicitAngularOffsets(msg)), std::runtime_error);

  // check single-element construction works

  msg.offsets.push_back(1.0);
  parsed = ExplicitAngularOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  parsed.AddOffset(2.0);
  parsed.FillMsg(tmpMsg); msg.offsets.push_back(2.0); EXPECT_EQ(msg, tmpMsg);

  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(2, parsed.Get(1));
  EXPECT_THROW(parsed.Get(2), std::out_of_range);

  parsed.AddOffset(2.0);
  parsed.FillMsg(tmpMsg); msg.offsets.push_back(2.0); EXPECT_EQ(msg, tmpMsg);

  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(1, parsed.Get(0));
  EXPECT_DOUBLE_EQ(2, parsed.Get(1));
  EXPECT_DOUBLE_EQ(2, parsed.Get(2));
  EXPECT_THROW(parsed.Get(3), std::out_of_range);
}

TEST(ScanLayout, TestRegularTimeOffsets)
{
  TimeOffsets msg, tmpMsg;
  msg.regular = true;
  msg.increment = ros::Duration(1.0);

  EXPECT_THROW((ExplicitTimeOffsets(msg)), std::runtime_error);

  RegularTimeOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(1));
  ASSERT_TRUE(parsed.HasLength(2));
  ASSERT_TRUE(parsed.HasLength(3));
  EXPECT_DOUBLE_EQ(0, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(1, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(2, parsed.Get(2).toSec());

  // time increment can be negative

  msg.increment = ros::Duration(-1);
  parsed = RegularTimeOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(3));
  EXPECT_DOUBLE_EQ(0, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(-2, parsed.Get(2).toSec());

  // zero duration is valid

  msg.increment = ros::Duration(0.0);
  parsed = RegularTimeOffsets(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(3));
  EXPECT_DOUBLE_EQ(0, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(0, parsed.Get(2).toSec());

  msg.increment = ros::Duration(1);
  msg.base_offset = ros::Duration(10);
  parsed = RegularTimeOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(3));
  EXPECT_DOUBLE_EQ(10, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(11, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(12, parsed.Get(2).toSec());

  ASSERT_THROW(parsed.AddOffset(ros::Duration(-1.5)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(-0.5)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(0.5)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(10.5)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(9.5)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(8)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(1)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(0)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(-1)), std::runtime_error);
  ASSERT_THROW(parsed.AddOffset(ros::Duration(-8)), std::runtime_error);

  parsed.AddOffset(ros::Duration(9));
  tmpMsg.offsets.push_back(ros::Duration(9)); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(4));
  EXPECT_DOUBLE_EQ( 9, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(10, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(11, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(12, parsed.Get(3).toSec());

  parsed.AddOffset(ros::Duration(10));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(4));
  EXPECT_DOUBLE_EQ( 9, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(10, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(11, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(12, parsed.Get(3).toSec());

  parsed.AddOffset(ros::Duration(9));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(4));
  EXPECT_DOUBLE_EQ( 9, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(10, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(11, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(12, parsed.Get(3).toSec());

  parsed.AddOffset(ros::Duration(8));
  tmpMsg.offsets.push_back(ros::Duration(8)); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ( 8, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 9, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(10, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(11, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ(12, parsed.Get(4).toSec());

  msg.increment = ros::Duration(1.5);
  msg.base_offset = ros::Duration(10.5);
  parsed = RegularTimeOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(12.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(13.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(15.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ(16.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(10.5));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(12.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(13.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(15.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ(16.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(12.0));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(12.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(13.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(15.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ(16.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(9.0));
  tmpMsg.offsets.push_back(ros::Duration(9.0)); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(6));
  EXPECT_DOUBLE_EQ( 9.0, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(12.0, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ(13.5, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ(15.0, parsed.Get(4).toSec());
  EXPECT_DOUBLE_EQ(16.5, parsed.Get(5).toSec());

  msg.increment = ros::Duration(-1.5);
  msg.base_offset = ros::Duration(10.5);
  parsed = RegularTimeOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 9.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ( 7.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 6.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 4.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(9.0));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 9.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ( 7.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 6.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 4.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(7.5));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(5));
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 9.0, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ( 7.5, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 6.0, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 4.5, parsed.Get(4).toSec());

  parsed.AddOffset(ros::Duration(12.0));
  parsed.FillMsg(tmpMsg); msg.base_offset = ros::Duration(12.0); EXPECT_EQ(msg, tmpMsg);
  ASSERT_TRUE(parsed.HasLength(6));
  EXPECT_DOUBLE_EQ(12.0, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(10.5, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ( 9.0, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 7.5, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 6.0, parsed.Get(4).toSec());
  EXPECT_DOUBLE_EQ( 4.5, parsed.Get(5).toSec());
}

TEST(ScanLayout, TestExplicitTimeOffsets)
{
  ros::Time::init();
  const auto now = ros::Time::now();

  TimeOffsets msg, tmpMsg;
  msg.regular = false;
  msg.offsets = {ros::Duration(-1.0), ros::Duration(1.0), ros::Duration(0.0),
                 ros::Duration(M_PI), now - ros::Time(0.0) };

  EXPECT_THROW((RegularTimeOffsets(msg)), std::runtime_error);

  ExplicitTimeOffsets parsed(msg);
  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(5, parsed.Length());
  ASSERT_TRUE(parsed.HasLength(5));
  ASSERT_FALSE(parsed.HasLength(4));
  ASSERT_FALSE(parsed.HasLength(6));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ(1, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(0, parsed.Get(2).toSec());
  EXPECT_NEAR(M_PI, parsed.Get(3).toSec(), 1e-6);
  EXPECT_DOUBLE_EQ(now.toSec(), parsed.Get(4).toSec());
  EXPECT_THROW(parsed.Get(5), std::out_of_range);

  // check empty construction doesn't succeed

  msg.offsets.clear();

  EXPECT_THROW((ExplicitTimeOffsets(msg)), std::runtime_error);

  // check single-element construction works

  msg.offsets.push_back(ros::Duration(1.0));
  parsed = ExplicitTimeOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(1, parsed.Length());
  ASSERT_TRUE(parsed.HasLength(1));
  EXPECT_DOUBLE_EQ(1, parsed.Get(0).toSec());
  EXPECT_THROW(parsed.Get(1), std::out_of_range);

  // check mutliple elements with the same value work

  msg.offsets = {ros::Duration(-1.0), ros::Duration(1.0), ros::Duration(-1.0),
                 ros::Duration(1.0), ros::Duration(1.0), ros::Duration(-1.0) };
  parsed = ExplicitTimeOffsets(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(6, parsed.Length());
  ASSERT_TRUE(parsed.HasLength(6));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(4).toSec());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(5).toSec());
  EXPECT_THROW(parsed.Get(6), std::out_of_range);

  parsed.AddOffset(ros::Duration(9));
  parsed.FillMsg(tmpMsg); msg.offsets.push_back(ros::Duration(9)); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(7, parsed.Length());
  ASSERT_TRUE(parsed.HasLength(7));
  EXPECT_DOUBLE_EQ(-1, parsed.Get(0).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(1).toSec());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(2).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(3).toSec());
  EXPECT_DOUBLE_EQ( 1, parsed.Get(4).toSec());
  EXPECT_DOUBLE_EQ(-1, parsed.Get(5).toSec());
  EXPECT_DOUBLE_EQ( 9, parsed.Get(6).toSec());
  EXPECT_THROW(parsed.Get(7), std::out_of_range);
}

TEST(ScanLayout, TestParsedScanLayout)
{
  ScanLayout msg, tmpMsg;
  msg.time_offsets.regular = false;
  msg.time_offsets.offsets = { ros::Duration(0.0), ros::Duration(1.0), ros::Duration(2.0) };
  msg.angular_offsets.regular = false;
  msg.angular_offsets.offsets = { -M_PI, 0, M_PI };

  ParsedScanLayout parsed(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(0.0, parsed.GetTime(0).toSec());
  EXPECT_DOUBLE_EQ(1.0, parsed.GetTime(1).toSec());
  EXPECT_DOUBLE_EQ(2.0, parsed.GetTime(2).toSec());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.GetAngle(0));
  EXPECT_DOUBLE_EQ(0, parsed.GetAngle(1));
  EXPECT_DOUBLE_EQ(M_PI, parsed.GetAngle(2));
  EXPECT_THROW(parsed.GetTime(3), std::out_of_range);
  EXPECT_THROW(parsed.GetAngle(3), std::out_of_range);

  parsed.AddOffset(1.0, ros::Duration(1.0));
  parsed.FillMsg(tmpMsg);
  msg.angular_offsets.offsets.push_back(1.0);
  msg.time_offsets.offsets.push_back(ros::Duration(1.0));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(4, parsed.Length());
  EXPECT_DOUBLE_EQ(0.0, parsed.GetTime(0).toSec());
  EXPECT_DOUBLE_EQ(1.0, parsed.GetTime(1).toSec());
  EXPECT_DOUBLE_EQ(2.0, parsed.GetTime(2).toSec());
  EXPECT_DOUBLE_EQ(1.0, parsed.GetTime(3).toSec());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.GetAngle(0));
  EXPECT_DOUBLE_EQ(0, parsed.GetAngle(1));
  EXPECT_DOUBLE_EQ(M_PI, parsed.GetAngle(2));
  EXPECT_DOUBLE_EQ(1, parsed.GetAngle(3));
  EXPECT_THROW(parsed.GetTime(4), std::out_of_range);
  EXPECT_THROW(parsed.GetAngle(4), std::out_of_range);

  msg.time_offsets.regular = true;
  msg.time_offsets.increment = ros::Duration(1.0);
  msg.time_offsets.base_offset = ros::Duration(10.0);
  msg.angular_offsets.regular = true;
  msg.angular_offsets.min = -M_PI;
  msg.angular_offsets.max = M_PI;
  msg.angular_offsets.increment = M_PI;
  msg.angular_offsets.exclude_last = true;

  parsed = ParsedScanLayout(msg);

  parsed.FillMsg(tmpMsg); EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(2, parsed.Length());
  EXPECT_DOUBLE_EQ(10.0, parsed.GetTime(0).toSec());
  EXPECT_DOUBLE_EQ(11.0, parsed.GetTime(1).toSec());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.GetAngle(0));
  EXPECT_DOUBLE_EQ(0, parsed.GetAngle(1));
  EXPECT_THROW(parsed.GetAngle(2), std::out_of_range);

  parsed.AddOffset(M_PI, ros::Duration(12.0));
  parsed.FillMsg(tmpMsg);
  msg.angular_offsets.offsets.push_back(M_PI);
  msg.time_offsets.offsets.push_back(ros::Duration(2.0));
  EXPECT_EQ(msg, tmpMsg);
  ASSERT_EQ(3, parsed.Length());
  EXPECT_DOUBLE_EQ(10.0, parsed.GetTime(0).toSec());
  EXPECT_DOUBLE_EQ(11.0, parsed.GetTime(1).toSec());
  EXPECT_DOUBLE_EQ(12.0, parsed.GetTime(2).toSec());
  EXPECT_DOUBLE_EQ(-M_PI, parsed.GetAngle(0));
  EXPECT_DOUBLE_EQ(0, parsed.GetAngle(1));
  EXPECT_DOUBLE_EQ(M_PI, parsed.GetAngle(2));
  EXPECT_THROW(parsed.GetAngle(3), std::out_of_range);
}

TEST(ScanLayout, TestMultiLayerLaserScanLayout)
{
  MultiLayerLaserScan msg;
  
  msg.scan_layout.time_offsets.regular = false;
  msg.scan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(1.0), ros::Duration(2.0) };
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = {-M_PI, 0, M_PI };

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets = {ros::Duration(0.0), ros::Duration(0.1), ros::Duration(0.2) };
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {-0.1 * M_PI, 0, 0.1 * M_PI };

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { -0.1, 0, 0.1 };

  // layout size must be equal to the number of elements in ranges
  EXPECT_THROW((MultiLayerLaserScanLayout(msg)), std::runtime_error);

  msg.ranges.resize(9, 0.0);

  MultiLayerLaserScanLayout parsed(msg);

  ASSERT_EQ(9, parsed.Length());

  EXPECT_DOUBLE_EQ(0.0, parsed.GetTime(0).toSec());
  EXPECT_DOUBLE_EQ(0.1, parsed.GetTime(1).toSec());
  EXPECT_DOUBLE_EQ(0.2, parsed.GetTime(2).toSec());
  EXPECT_DOUBLE_EQ(1.0, parsed.GetTime(3).toSec());
  EXPECT_DOUBLE_EQ(1.1, parsed.GetTime(4).toSec());
  EXPECT_DOUBLE_EQ(1.2, parsed.GetTime(5).toSec());
  EXPECT_DOUBLE_EQ(2.0, parsed.GetTime(6).toSec());
  EXPECT_DOUBLE_EQ(2.1, parsed.GetTime(7).toSec());
  EXPECT_DOUBLE_EQ(2.2, parsed.GetTime(8).toSec());

  EXPECT_DOUBLE_EQ(-M_PI - 0.1, parsed.GetScanAngle(0));
  EXPECT_DOUBLE_EQ(-0.1 * M_PI, parsed.GetSubscanAngle(0));
  EXPECT_DOUBLE_EQ(-M_PI, parsed.GetScanAngle(1));
  EXPECT_DOUBLE_EQ(0.0, parsed.GetSubscanAngle(1));
  EXPECT_DOUBLE_EQ(-M_PI + 0.1, parsed.GetScanAngle(2));
  EXPECT_DOUBLE_EQ(0.1 * M_PI, parsed.GetSubscanAngle(2));
  EXPECT_DOUBLE_EQ(0.0 - 0.1, parsed.GetScanAngle(3));
  EXPECT_DOUBLE_EQ(-0.1 * M_PI, parsed.GetSubscanAngle(3));
  EXPECT_DOUBLE_EQ(0.0, parsed.GetScanAngle(4));
  EXPECT_DOUBLE_EQ(0.0, parsed.GetSubscanAngle(4));
  EXPECT_DOUBLE_EQ(0.0 + 0.1, parsed.GetScanAngle(5));
  EXPECT_DOUBLE_EQ(0.1 * M_PI, parsed.GetSubscanAngle(5));
  EXPECT_DOUBLE_EQ(M_PI - 0.1, parsed.GetScanAngle(6));
  EXPECT_DOUBLE_EQ(-0.1 * M_PI, parsed.GetSubscanAngle(6));
  EXPECT_DOUBLE_EQ(M_PI, parsed.GetScanAngle(7));
  EXPECT_DOUBLE_EQ(0.0, parsed.GetSubscanAngle(7));
  EXPECT_DOUBLE_EQ(M_PI + 0.1, parsed.GetScanAngle(8));
  EXPECT_DOUBLE_EQ(0.1 * M_PI, parsed.GetSubscanAngle(8));

  EXPECT_THROW(parsed.GetTime(9), std::out_of_range);
  EXPECT_THROW(parsed.GetScanAngle(9), std::out_of_range);
  EXPECT_THROW(parsed.GetSubscanAngle(9), std::out_of_range);

  for (size_t i = 0; i < 9; ++i)
  {
    double scanAngle, subscanAngle;
    ros::Duration time;
    parsed.GetAll(i, scanAngle, subscanAngle, time);
    EXPECT_EQ(parsed.GetScanAngle(i), scanAngle);
    EXPECT_EQ(parsed.GetSubscanAngle(i), subscanAngle);
    EXPECT_EQ(parsed.GetTime(i), time);
  }
}

TEST(RealScanners, SickLMS151AsScan)
{
  // a single-layer lidar, but it should be possible to represent it
  // as 541 single-element subscans

  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.scan_layout.time_offsets.regular = true;
  msg.scan_layout.time_offsets.increment = ros::Duration(2.77777780866e-05);
  msg.scan_layout.angular_offsets.regular = true;
  msg.scan_layout.angular_offsets.min = -2.35619449615;
  msg.scan_layout.angular_offsets.max = 2.35619449615;
  msg.scan_layout.angular_offsets.increment = 0.00872664619237;

  msg.subscan_layout.time_offsets.regular = true;
  msg.subscan_layout.time_offsets.increment = ros::Duration(0);
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = { 0 };

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = { 0 };

  msg.ranges.resize(541, 0.0);
  msg.intensities.resize(541, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(541, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  for (size_t i = 0; i < 541 && it != it.end(); ++i, ++it)
  {
    EXPECT_DOUBLE_EQ(0.0, *(*it).range);
    EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);
    EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
    EXPECT_NEAR(i * 0.00872664619237 - 2.35619449615 , (*it).scanAngle, 10e-6);
    EXPECT_NEAR(i * 2.77777780866e-05 + 10.0 , (*it).timestamp.toSec(), 10e-4);
  }
}

TEST(RealScanners, SickLMS151AsSubcan)
{
  // a single-layer lidar, but it should be possible to represent it
  // as one 541-element subscan with zero vertical angle increment

  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.scan_layout.time_offsets.regular = true;
  msg.scan_layout.time_offsets.increment = ros::Duration(0);
  msg.scan_layout.angular_offsets.regular = false;
  msg.scan_layout.angular_offsets.offsets = { -2.35619449615 };

  msg.subscan_layout.time_offsets.regular = true;
  msg.subscan_layout.time_offsets.increment = ros::Duration(2.77777780866e-05);
  msg.subscan_layout.angular_offsets.regular = true;
  msg.subscan_layout.angular_offsets.min = 0;
  msg.subscan_layout.angular_offsets.max = 0;
  msg.subscan_layout.angular_offsets.samples = 541;

  msg.scan_offsets_during_subscan.regular = true;
  msg.scan_offsets_during_subscan.min = 0;
  msg.scan_offsets_during_subscan.max = 2 * 2.35619449615;
  msg.scan_offsets_during_subscan.increment = 0.00872664619237;

  msg.ranges.resize(541, 0.0);
  msg.intensities.resize(541, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(541, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  for (size_t i = 0; i < 541 && it != it.end(); ++i, ++it)
  {
    EXPECT_DOUBLE_EQ(0.0, *(*it).range);
    EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);
    EXPECT_DOUBLE_EQ(0.0, (*it).subscanAngle);
    EXPECT_NEAR(i * 0.00872664619237 - 2.35619449615, (*it).scanAngle, 10e-6);
    EXPECT_NEAR(i * 2.77777780866e-05 + 10.0 , (*it).timestamp.toSec(), 10e-4);
  }
}

TEST(RealScanners, VelodyneHDL32ERegular)
{
  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.subscan_layout.time_offsets.regular = true;
  msg.subscan_layout.time_offsets.increment = ros::Duration(1.152e-6);
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {
      -30.67, -9.33, -29.33, -8.00, -28.00, -6.67, -26.67, -5.33,
      -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33,  0.00,
      -20.00,  1.33, -18.67,  2.67, -17.33,  4.00, -16.00,  5.33,
      -14.67,  6.67, -13.33,  8.00, -12.00,  9.33, -10.67,  10.67
  };
  for (size_t i = 0; i < 32; ++i)
    msg.subscan_layout.angular_offsets.offsets[i] *= 2 * M_PI / 360;

  msg.scan_layout.time_offsets.regular = true;
  // 40 pulses per subscan
  msg.scan_layout.time_offsets.increment = msg.subscan_layout.time_offsets.increment * 40;
  msg.scan_layout.angular_offsets.regular = true;
  msg.scan_layout.angular_offsets.min = 0;
  msg.scan_layout.angular_offsets.max = 2 * M_PI;
  // 10 Hz gives 181 packets per scan, each packet holding 12 subscans, i.e. 2172 subscans per scan
  msg.scan_layout.angular_offsets.samples = 2172;
  msg.scan_layout.angular_offsets.exclude_last = true;

  msg.scan_offsets_during_subscan.regular = true;
  msg.scan_offsets_during_subscan.min = 0;
  msg.scan_offsets_during_subscan.max = 2 * M_PI / msg.scan_layout.angular_offsets.samples / 40 * 32;
  msg.scan_offsets_during_subscan.samples = 32;

  const auto numPoints = 69504;

  msg.ranges.resize(numPoints, 0.0);
  msg.intensities.resize(numPoints, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(numPoints, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  const auto velocity = msg.scan_offsets_during_subscan.max / 32;
  const auto timeIncrement = msg.subscan_layout.time_offsets.increment;
  const auto timeOffset = msg.header.stamp - ros::Time(0, 0);

  for (size_t i = 0; i < numPoints && it != it.end(); ++i, ++it)
  {
    const auto scan = i / 32;
    const auto subScan = i % 32;

    EXPECT_DOUBLE_EQ(0.0, *(*it).range);
    EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);
    EXPECT_DOUBLE_EQ(msg.subscan_layout.angular_offsets.offsets[i % 32], (*it).subscanAngle);
    EXPECT_NEAR(scan * velocity * 40 + subScan * velocity, (*it).scanAngle, 1.1 * velocity);
    EXPECT_NEAR((timeIncrement * scan * 40 + timeIncrement * subScan + timeOffset).toSec(),
                (*it).timestamp.toSec(), 1.1 * timeIncrement.toSec());
  }
}

TEST(RealScanners, VelodyneHDL32EData)
{
  // passed by CMake
  const std::string dataDir = DATA_DIR;

  // this file was generated manually by editing the velodyne_pointcloud source
  // and capturing all azimuths and elevations computed by the driver during one
  // scan
  std::ifstream file(dataDir + "/velodyne_hdl_32e.csv");

  // <elevation, azimuth>
  std::vector<std::pair<double, double> > dataList;

  std::string line;
  // Iterate through each line and split the content using delimeter
  while (getline(file, line))
  {
    std::vector<std::string> vec;
    boost::algorithm::split(vec, line, boost::is_any_of(","));
    dataList.emplace_back(atof(vec[0].c_str()), atof(vec[1].c_str()));
  }
  // Close the File
  file.close();

  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.subscan_layout.time_offsets.regular = true;
  msg.subscan_layout.time_offsets.increment = ros::Duration(1.152e-6);
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {
      -30.67, -9.33, -29.33, -8.00, -28.00, -6.67, -26.67, -5.33,
      -25.33, -4.00, -24.00, -2.67, -22.67, -1.33, -21.33,  0.00,
      -20.00,  1.33, -18.67,  2.67, -17.33,  4.00, -16.00,  5.33,
      -14.67,  6.67, -13.33,  8.00, -12.00,  9.33, -10.67,  10.67
  };
  for (size_t i = 0; i < 32; ++i)
    msg.subscan_layout.angular_offsets.offsets[i] *= 2 * M_PI / 360;

  msg.scan_layout.time_offsets.regular = true;
  // we're not interested in time in this test
  msg.scan_layout.time_offsets.increment = ros::Duration(0);
  msg.scan_layout.angular_offsets.regular = false;
  for (size_t i = 0; i < dataList.size(); i += msg.subscan_layout.angular_offsets.offsets.size())
    msg.scan_layout.angular_offsets.offsets.push_back(dataList[i].second);

  msg.scan_offsets_during_subscan.regular = true;
  msg.scan_offsets_during_subscan.min = 0;
  msg.scan_offsets_during_subscan.max = 2 * M_PI / 2172 / 40 * msg.subscan_layout.angular_offsets.offsets.size();
  msg.scan_offsets_during_subscan.samples = msg.subscan_layout.angular_offsets.offsets.size();
  msg.scan_offsets_during_subscan.increment = msg.scan_offsets_during_subscan.max / (msg.scan_offsets_during_subscan.samples - 1);

  const auto numPoints = dataList.size();

  msg.ranges.resize(numPoints, 0.0);
  msg.intensities.resize(numPoints, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(numPoints, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  for (size_t i = 0; i < numPoints && it != it.end(); )
  {
    for (size_t j = 0; j < msg.subscan_layout.angular_offsets.offsets.size(); ++j, ++i, ++it)
    {
      EXPECT_DOUBLE_EQ(0.0, *(*it).range);
      EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);

      const auto subscanAngle = dataList[i].first;
      const auto scanAngle = dataList[i].second + j * msg.scan_offsets_during_subscan.increment;

      EXPECT_NEAR(subscanAngle, (*it).subscanAngle, 1e-4);
      EXPECT_NEAR(msg.subscan_layout.angular_offsets.offsets[j], (*it).subscanAngle, 1e-4);
      EXPECT_NEAR(scanAngle, (*it).scanAngle, 1e-4);
      EXPECT_NEAR(1.152e-6 * j, ((*it).timestamp - msg.header.stamp).toSec(), 1e-6);
    }
  }

}

TEST(RealScanners, Ouster64Regular)
{
  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.subscan_layout.time_offsets.regular = true;
  msg.subscan_layout.time_offsets.increment = ros::Duration(0); // solid-state
  msg.subscan_layout.angular_offsets.regular = true;
  msg.subscan_layout.angular_offsets.min = -16.611 * 2 * M_PI / 360;
  msg.subscan_layout.angular_offsets.max =  16.611 * 2 * M_PI / 360;
  msg.subscan_layout.angular_offsets.samples = -64;

  msg.scan_layout.time_offsets.regular = true;
  // 40 pulses per subscan
  msg.scan_layout.time_offsets.increment = ros::Duration(0.1 / 2048);
  msg.scan_layout.angular_offsets.regular = true;
  msg.scan_layout.angular_offsets.min = 0;
  msg.scan_layout.angular_offsets.max = 2 * M_PI;
  msg.scan_layout.angular_offsets.exclude_last = true;
  msg.scan_layout.angular_offsets.samples = 2048;

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = {
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
      3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
  };
  for (size_t i = 0; i < 64; ++i)
    msg.scan_offsets_during_subscan.offsets[i] *= 2 * M_PI / 360;

  std::vector<double> beam_altitude_angles = {
      16.611,  16.084,  15.557,  15.029,  14.502,  13.975,  13.447,  12.920,
      12.393,  11.865,  11.338,  10.811,  10.283,  9.756,   9.229,   8.701,
      8.174,   7.646,   7.119,   6.592,   6.064,   5.537,   5.010,   4.482,
      3.955,   3.428,   2.900,   2.373,   1.846,   1.318,   0.791,   0.264,
      -0.264,  -0.791,  -1.318,  -1.846,  -2.373,  -2.900,  -3.428,  -3.955,
      -4.482,  -5.010,  -5.537,  -6.064,  -6.592,  -7.119,  -7.646,  -8.174,
      -8.701,  -9.229,  -9.756,  -10.283, -10.811, -11.338, -11.865, -12.393,
      -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
  };
  for (size_t i = 0; i < 64; ++i)
    beam_altitude_angles[i] *= 2 * M_PI / 360;


  const auto numPoints = 2048 * 64;

  msg.ranges.resize(numPoints, 0.0);
  msg.intensities.resize(numPoints, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(numPoints, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  for (size_t i = 0; i < numPoints && it != it.end(); ++i, ++it)
  {
    const auto scan = i / 64;
    const auto subScan = i % 64;

    const auto baseScanAngle = 2 * M_PI / 2048 * scan;
    const auto scanAngle = baseScanAngle + msg.scan_offsets_during_subscan.offsets[subScan];

    EXPECT_DOUBLE_EQ(0.0, *(*it).range);
    EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);
    EXPECT_NEAR(beam_altitude_angles[subScan], (*it).subscanAngle, 0.1 * 2 * M_PI / 360);
    EXPECT_NEAR(scanAngle, (*it).scanAngle, 0.1 * 3.164 * 2 * M_PI / 360);
    EXPECT_NEAR(10 + (msg.scan_layout.time_offsets.increment * scan).toSec(),
                (*it).timestamp.toSec(), 1.1 * msg.scan_layout.time_offsets.increment.toSec());
  }
}

TEST(RealScanners, RSLidar32Regular)
{
  MultiLayerLaserScan msg;

  msg.header.stamp = ros::Time(10.0);

  msg.subscan_layout.time_offsets.regular = false;
  msg.subscan_layout.time_offsets.offsets.resize(32);
  for (size_t i = 0; i < 16; ++i)
  {
    const ros::Duration offset(0, i * 3000);
    msg.subscan_layout.time_offsets.offsets[i] = offset;
    msg.subscan_layout.time_offsets.offsets[i + 16] = offset;
  }
  msg.subscan_layout.angular_offsets.regular = false;
  msg.subscan_layout.angular_offsets.offsets = {
      -10.2637,   0.2972,  -6.4063,  -0.0179,   2.2794,  -0.3330,   3.2973,  -0.6670,
        4.6136,   1.6849,   7.0176,   1.2972,  10.2983,   1.0000,  15.0167,   0.7028,
      -25.0000,  -2.2794, -14.6380,  -2.6670,  -7.9100,  -3.0179,  -5.4070,  -3.2973,
       -3.6492,  -1.0179,  -4.0534,  -1.3330,  -4.3864,  -1.6312,  -4.6492,  -1.9821
  };
  for (size_t i = 0; i < 32; ++i)
    msg.subscan_layout.angular_offsets.offsets[i] *= 2 * M_PI / 360;

  msg.scan_layout.time_offsets.regular = true;
  // 40 pulses per subscan
  msg.scan_layout.time_offsets.increment = ros::Duration(0, 50000);
  msg.scan_layout.angular_offsets.regular = true;
  msg.scan_layout.angular_offsets.min = 0;
  msg.scan_layout.angular_offsets.max = 2 * M_PI;
  msg.scan_layout.angular_offsets.exclude_last = true;
  msg.scan_layout.angular_offsets.samples = 1500;

  msg.scan_offsets_during_subscan.regular = false;
  msg.scan_offsets_during_subscan.offsets = {
      9.1205,  -7.1203,   8.9109,  -1.7781,   9.1205,   3.5716,  -6.8380,   8.7856,
      9.1205,  -7.1555,  -6.7674,  -1.8139,   9.0507,   3.4859,  -6.9439,   8.9007,
     -6.6968,  -7.3317,  -6.6614,  -2.0000,  -6.7674,   3.3932,  -6.8380,   8.8060,
     -7.1908,  -7.2965,  -1.8643,  -1.8139,   3.4932,   3.5716,   8.8060,   8.7308
  };
  for (size_t i = 0; i < 32; ++i)
    msg.scan_offsets_during_subscan.offsets[i] *= 2 * M_PI / 360;

  const auto numPoints = 1500 * 32;

  msg.ranges.resize(numPoints, 0.0);
  msg.intensities.resize(numPoints, 0.0);

  const auto parsed = std::make_shared<MultiLayerLaserScanLayout>(msg);

  ASSERT_EQ(numPoints, parsed->Length());

  MultiLayerLaserScanBaseFieldsConstIterator it(msg, parsed);

  for (size_t i = 0; i < numPoints && it != it.end(); ++i, ++it)
  {
    const auto scan = i / 32;
    const auto subScan = i % 32;

    const auto baseScanAngle = 2 * M_PI / 1500 * scan;
    const auto scanAngle = baseScanAngle + msg.scan_offsets_during_subscan.offsets[subScan];

    EXPECT_DOUBLE_EQ(0.0, *(*it).range);
    EXPECT_DOUBLE_EQ(0.0, *(*it).intensity);
    EXPECT_NEAR(msg.subscan_layout.angular_offsets.offsets[subScan], (*it).subscanAngle, 0.1 * 2 * M_PI / 360);
    EXPECT_NEAR(scanAngle, (*it).scanAngle, 0.1 * 2 * M_PI / 360);
    EXPECT_NEAR(10 + (msg.scan_layout.time_offsets.increment * scan + msg.subscan_layout.time_offsets.offsets[subScan]).toSec(),
                (*it).timestamp.toSec(), 1.1 * msg.scan_layout.time_offsets.increment.toSec());
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}