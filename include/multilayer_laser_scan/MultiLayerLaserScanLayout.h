#ifndef MULTILAYER_LASER_SCAN_MULTILAYERLASERSCANLAYOUT_H
#define MULTILAYER_LASER_SCAN_MULTILAYERLASERSCANLAYOUT_H

#include <multilayer_laser_scan/ScanLayout.h>
#include <multilayer_laser_scan/MultiLayerLaserScan.h>

namespace sensor_msgs
{

struct ParsedAngularOffsets
{
  virtual double Get(size_t i) const = 0;
  virtual size_t Length() const = 0;
  virtual void AddOffset(double offset) = 0;
  virtual void FillMsg(AngularOffsets& msg) const = 0;
};

class RegularAngularOffsets : public ParsedAngularOffsets
{
  public: explicit RegularAngularOffsets(const AngularOffsets& _msg);
  public: double Get(size_t i) const override;
  public: size_t Length() const override;
  public: void AddOffset(double offset) override;
  public: void FillMsg(AngularOffsets& msg) const override;

  private: inline double FirstAngle() const;
  private: inline double LastAngle() const;
  private: inline double RealMaxAngle() const;
  private: inline double RealMinAngle() const;

  protected: double angleMin;
  protected: double angleMax;
  protected: double angleIncrement;
  protected: bool excludeLast;

  protected: int samples;
};

class ExplicitAngularOffsets : public ParsedAngularOffsets
{
  public: explicit ExplicitAngularOffsets(const AngularOffsets &_msg);
  public: double Get(size_t i) const override;
  public: size_t Length() const override;
  public: void AddOffset(double offset) override;
  public: void FillMsg(AngularOffsets& msg) const override;

  protected: std::vector<double> offsets;
};


struct ParsedTimeOffsets
{
  virtual ros::Duration Get(size_t i) const = 0;
  virtual void AddOffset(const ros::Duration& offset) = 0;
  virtual bool HasLength(size_t length) const = 0;
  virtual void FillMsg(TimeOffsets& msg) const = 0;
};

class RegularTimeOffsets : public ParsedTimeOffsets
{
  public: explicit RegularTimeOffsets(const TimeOffsets& _msg);
  public: ros::Duration Get(size_t i) const override;
  public: void AddOffset(const ros::Duration& offset) override;
  public: bool HasLength(size_t length) const override;
  public: void FillMsg(TimeOffsets& msg) const override;

  protected: ros::Duration baseOffset;
  protected: ros::Duration timeIncrement;
};

class ExplicitTimeOffsets : public ParsedTimeOffsets
{
  public: explicit ExplicitTimeOffsets(const TimeOffsets &_msg);
  public: ros::Duration Get(size_t i) const override;
  public: size_t Length() const;
  public: void AddOffset(const ros::Duration& offset) override;
  public: bool HasLength(size_t length) const override;
  public: void FillMsg(TimeOffsets& msg) const override;

  protected: std::vector<ros::Duration> offsets;
};

class ParsedScanLayout
{
  public: explicit ParsedScanLayout(const ScanLayout& _msg);
  public: virtual double GetAngle(size_t i) const;
  public: virtual ros::Duration GetTime(size_t i) const;
  public: virtual size_t Length() const;
  public: virtual void AddOffset(double angularOffset, const ros::Duration& timeOffset);
  public: virtual void FillMsg(ScanLayout& msg) const;

  protected: std::unique_ptr<ParsedAngularOffsets> angularOffsets;
  protected: std::unique_ptr<ParsedTimeOffsets> timeOffsets;
};

class MultiLayerLaserScanLayout
{
  public: explicit MultiLayerLaserScanLayout(const MultiLayerLaserScan& _msg);
  public: virtual double GetScanAngle(size_t i) const;
  public: virtual double GetSubscanAngle(size_t i) const;
  public: virtual ros::Duration GetTime(size_t i) const;
  public: virtual void GetAll(size_t i, double& _scanAngle,
      double& _subscanAngle, ros::Duration& _time) const;
  public: virtual size_t Length() const;
  public: virtual void FillMsg(MultiLayerLaserScan& msg) const;

  protected: virtual size_t GetScanIndex(size_t i) const;
  protected: virtual size_t GetSubscanIndex(size_t i) const;

  protected: virtual double GetScanAngleByIndex(size_t scanIndex, size_t subscanIndex) const;
  protected: virtual double GetSubscanAngleByIndex(size_t i) const;
  protected: virtual ros::Duration GetTimeByIndex(size_t scanIndex, size_t subscanIndex) const;

  protected: ParsedScanLayout subscanLayout;
  protected: ParsedScanLayout scanLayout;
  protected: std::unique_ptr<ParsedAngularOffsets> scanAngularVelocity;
  protected: size_t length;
  protected: size_t subscanLength;
};

}

#endif //MULTILAYER_LASER_SCAN_MULTILAYERLASERSCANLAYOUT_H
