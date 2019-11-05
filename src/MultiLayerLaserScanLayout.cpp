#include <multilayer_laser_scan/MultiLayerLaserScanLayout.h>

#include <ros/console.h>

namespace sensor_msgs
{

RegularAngularOffsets::RegularAngularOffsets(const AngularOffsets& _msg)
{
  if (!_msg.regular)
    throw std::runtime_error("Trying to parse explicit angular offsets as regular ones.");

  if (_msg.increment == 0.0 && _msg.samples == 0)
    throw std::runtime_error("Both increment and samples cannot be zero.");

  this->angleMin = _msg.min;
  this->angleMax = _msg.max;
  this->excludeLast = _msg.exclude_last;

  if (this->angleMin > this->angleMax + 1e-9)
    throw std::runtime_error("Minimum angle is larger than the maximum angle.");

  // degenerative case with min == max, but we have to check if the user didn't
  // want to specify full circle instead
  if (this->angleMin == this->angleMax)
  {
    if (_msg.samples == 1 || _msg.samples == -1)
    {
      if (this->excludeLast)
        throw std::runtime_error("AngleOffsets: min == max, abs(samples) == 1 and exclude_last is true");

      this->angleIncrement = 0.0;
      this->samples = _msg.samples;
      return;
    }
    else if (_msg.increment != 0.0)
    {
      if (_msg.samples > 1)
      {
        this->angleMax += 2 * M_PI;
      }
      else if (_msg.samples < -1)
      {
        this->angleMin -= 2 * M_PI;
      }
      else // _msg.samples == 0
      {
        if (this->excludeLast)
          throw std::runtime_error("AngleOffsets: min == max, samples == 0 and exclude_last is true");

        this->angleIncrement = 0.0;
        this->samples = 1;
        return;
      }
    }
  }

  const auto range = this->angleMax - this->angleMin;

  if (_msg.increment != 0.0)
  {
    this->angleIncrement = _msg.increment;

    const auto lengthAsDouble = range / std::abs(this->angleIncrement);
    const auto rangeIsMultipleOfIncrements =
        std::abs(lengthAsDouble - round(lengthAsDouble)) <= 1e-9;

    this->samples = static_cast<int>(trunc(range / this->angleIncrement));

    if (rangeIsMultipleOfIncrements)
    {
      if (!this->excludeLast)
        this->samples += (_msg.increment >= 0 ? 1 : -1);
    }
    else
    {
      this->samples += (_msg.increment >= 0 ? 1 : -1);

      ROS_WARN_STREAM_ONCE(
        "RegularAngleOffsets: range " << range << " is not divisible by "
        "angle increment " << this->angleIncrement << ". Only processing those "
        "angles lower than max angle that are multiples of angle increment.");
    }
  }

  // not else if, because we want to check if increment and samples yield
  // comparable values in case both are specified
  if (_msg.samples != 0)
  {
    double increment;

    if (_msg.samples == 1 || _msg.samples == -1)
    {
      increment = range * _msg.samples; // don't forget to keep sign
    }
    else
    {
      auto samplesCorrected = _msg.samples;
      if (_msg.samples > 0 && !this->excludeLast)
        samplesCorrected -=  1;
      else if (_msg.samples < 0 && !this->excludeLast)
        samplesCorrected += 1;

      increment = range / samplesCorrected;
    }

    if (_msg.increment == 0.0)
    {
      this->angleIncrement = increment;
      this->samples = _msg.samples;
    }
    else if (std::abs(increment - this->angleIncrement) > 1e-9)
    {
      ROS_WARN_STREAM_ONCE("Number of samples specified in message ("
        << _msg.samples << ") is different from the number of samples "
        << "computed from increment (" << this->samples << "). Giving priority "
        << "to the latter.");
    }
  }
}

double RegularAngularOffsets::Get(size_t i) const
{
  if (i >= std::abs(this->samples))
    throw std::out_of_range("Requested element past the end of angular offsets.");

  return this->FirstAngle() + i * this->angleIncrement;
}

size_t RegularAngularOffsets::Length() const
{
  return static_cast<size_t>(std::abs(this->samples));
}

void RegularAngularOffsets::AddOffset(double offset)
{
  if (this->excludeLast && std::abs(offset - (this->LastAngle() + this->angleIncrement)) < 1e-6)
  {
    this->excludeLast = false;
    this->samples += (this->angleIncrement >= 0 ? 1 : -1);
  }
  else if (offset > this->angleMax &&
    std::abs((this->RealMaxAngle() + std::abs(this->angleIncrement)) - offset) < 1e-6)
  {
    this->angleMax = offset;
    this->excludeLast = false;
    this->samples += (this->angleIncrement >= 0 ? 1 : -1);
  }
  else if (offset < this->angleMin &&
    std::abs((this->RealMinAngle() - std::abs(this->angleIncrement)) - offset) < 1e-6)
  {
    this->angleMin = offset;
    this->samples += (this->angleIncrement >= 0 ? 1 : -1);
  }
  else
  {
    throw std::runtime_error("Regular angular offsets can only be resized by "
                             "angle increment and outside of the current "
                             "angular range.");
  }
}

void RegularAngularOffsets::FillMsg(AngularOffsets& msg) const
{
  msg.regular = true;
  msg.increment = this->angleIncrement;
  msg.samples = this->samples;
  msg.min = this->angleMin;
  msg.max = this->angleMax;
  msg.exclude_last = this->excludeLast;
}

double RegularAngularOffsets::FirstAngle() const
{
  // if angleIncrement is negative, we go backwards from angleMax to angleMin
  return (this->angleIncrement >= 0) ? this->angleMin : this->angleMax;
}

double RegularAngularOffsets::LastAngle() const
{
  return this->Get(this->Length() - 1);
}

double RegularAngularOffsets::RealMaxAngle() const
{
  return (this->angleIncrement >= 0) ? this->LastAngle() : this->angleMax;;
}

double RegularAngularOffsets::RealMinAngle() const
{
  return (this->angleIncrement >= 0) ? this->angleMin : this->LastAngle();;
}

ExplicitAngularOffsets::ExplicitAngularOffsets(const AngularOffsets &_msg)
{
  if (_msg.regular)
    throw std::runtime_error("Trying to parse regular angular offsets as explicit ones.");

  if (_msg.offsets.empty())
    throw std::runtime_error("Empty explicit angular offsets are invalid.");

  this->offsets = _msg.offsets;
}

double ExplicitAngularOffsets::Get(size_t i) const
{
  return this->offsets.at(i);
}

size_t ExplicitAngularOffsets::Length() const
{
  return this->offsets.size();
}

void ExplicitAngularOffsets::AddOffset(double offset)
{
  this->offsets.push_back(offset);
}

void ExplicitAngularOffsets::FillMsg(AngularOffsets &msg) const
{
  msg.regular = false;
  msg.offsets = this->offsets;
}

RegularTimeOffsets::RegularTimeOffsets(const TimeOffsets &_msg)
{
  if (!_msg.regular)
    throw std::runtime_error("Trying to parse explicit time offsets as regular ones");

  this->baseOffset = _msg.base_offset;
  this->timeIncrement = _msg.increment;
}

ros::Duration RegularTimeOffsets::Get(size_t i) const
{
  return this->timeIncrement * i + this->baseOffset;
}

void RegularTimeOffsets::AddOffset(const ros::Duration& offset)
{
  const auto first = this->Get(0);
  const auto beforeFirst = first - this->timeIncrement;

  const auto eps = 1e-6;
  if (std::abs((offset - beforeFirst).toSec()) < eps)
  {
    this->baseOffset = offset;
    return;
  }

  const auto positiveIncrement = this->timeIncrement.sec >= 0;
  const auto epsDuration = ros::Duration(eps);

  if ((positiveIncrement && offset < this->baseOffset - epsDuration) ||
      (!positiveIncrement && offset > this->baseOffset + epsDuration))
  {
    throw std::runtime_error("Adding offsets to regular time offsets can only "
      "be done so that the added offset is current base offset minus exactly "
      "one time increment.");
  }

  const auto multiples = std::abs(offset.toSec() / this->timeIncrement.toSec());

  if (std::abs(std::round(multiples) - multiples) > eps)
  {
    throw std::runtime_error("Regular time offsets can only be extended by "
                             "time increment multiples.");
  }

  // do nothing in case a correct multiple was passed, since the offsets
  // structure is an endless generator in the direction of time offset
}

bool RegularTimeOffsets::HasLength(const size_t /*length*/) const
{
  // this is an endless generator
  return true;
}

void RegularTimeOffsets::FillMsg(TimeOffsets& msg) const
{
  msg.regular = true;
  msg.base_offset = this->baseOffset;
  msg.increment = this->timeIncrement;
}

ExplicitTimeOffsets::ExplicitTimeOffsets(const TimeOffsets &_msg)
{
  if (_msg.regular)
    throw std::runtime_error("Trying to parse regular time offsets as explicit ones");

  if (_msg.offsets.empty())
    throw std::runtime_error("Time offsets cannot be empty.");

  this->offsets = _msg.offsets;
}

ros::Duration ExplicitTimeOffsets::Get(size_t i) const
{
  return this->offsets.at(i);
}

size_t ExplicitTimeOffsets::Length() const
{
  return this->offsets.size();
}

void ExplicitTimeOffsets::AddOffset(const ros::Duration& offset)
{
  this->offsets.emplace_back(offset.sec, offset.nsec);
}

bool ExplicitTimeOffsets::HasLength(const size_t length) const
{
  return this->Length() == length;
}

void ExplicitTimeOffsets::FillMsg(TimeOffsets &msg) const
{
  msg.regular = false;
  msg.offsets = this->offsets;
}

ParsedScanLayout::ParsedScanLayout(const ScanLayout& _msg)
{
  if (_msg.angular_offsets.regular)
    this->angularOffsets.reset(new RegularAngularOffsets(_msg.angular_offsets));
  else
    this->angularOffsets.reset(new ExplicitAngularOffsets(_msg.angular_offsets));

  if (_msg.time_offsets.regular)
    this->timeOffsets.reset(new RegularTimeOffsets(_msg.time_offsets));
  else
    this->timeOffsets.reset(new ExplicitTimeOffsets(_msg.time_offsets));

  if (!this->timeOffsets->HasLength(this->angularOffsets->Length()))
    throw std::runtime_error("Angular offsets do not have the same number of elements as time offsets");
}

double ParsedScanLayout::GetAngle(size_t i) const
{
  return this->angularOffsets->Get(i);
}

ros::Duration ParsedScanLayout::GetTime(size_t i) const
{
  return this->timeOffsets->Get(i);
}

size_t ParsedScanLayout::Length() const
{
  return this->angularOffsets->Length();
}

void ParsedScanLayout::AddOffset(double angularOffset, const ros::Duration &timeOffset)
{
  this->angularOffsets->AddOffset(angularOffset);
  this->timeOffsets->AddOffset(timeOffset);
}

void ParsedScanLayout::FillMsg(ScanLayout& msg) const
{
  this->angularOffsets->FillMsg(msg.angular_offsets);
  this->timeOffsets->FillMsg(msg.time_offsets);
}

MultiLayerLaserScanLayout::MultiLayerLaserScanLayout(const MultiLayerLaserScan& _msg) :
  subscanLayout(ParsedScanLayout(_msg.subscan_layout)),
  scanLayout(ParsedScanLayout(_msg.scan_layout))
{
  this->subscanLength = this->subscanLayout.Length();
  this->length = this->scanLayout.Length() * this->subscanLength;

  if (_msg.scan_offsets_during_subscan.regular)
    this->scanAngularVelocity.reset(new RegularAngularOffsets(_msg.scan_offsets_during_subscan));
  else
    this->scanAngularVelocity.reset(new ExplicitAngularOffsets(_msg.scan_offsets_during_subscan));

  if (this->scanAngularVelocity->Length() != this->subscanLength)
    throw std::runtime_error("Length of scan_offsets_during_subscan " +
      std::to_string(this->scanAngularVelocity->Length()) + " is not the "
      "same as length of subscans " + std::to_string(this->subscanLength));

  if (this->length != _msg.ranges.size())
    throw std::runtime_error("Scan layout " + std::to_string(this->length) +
      " size doesn't correspond to the number of actual points " +
      std::to_string(_msg.ranges.size()));
}

double MultiLayerLaserScanLayout::GetScanAngleByIndex(
    const size_t scanIndex, const size_t subscanIndex) const
{
  const auto baseAngle = this->scanLayout.GetAngle(scanIndex);
  const auto offsetAngle = this->scanAngularVelocity->Get(subscanIndex);

  return baseAngle + offsetAngle;
}

double MultiLayerLaserScanLayout::GetScanAngle(const size_t i) const
{
  if (i >= this->length)
    throw std::out_of_range("Requested point is outside of the current layout.");

  return this->GetScanAngleByIndex(this->GetScanIndex(i), this->GetSubscanIndex(i));
}

double MultiLayerLaserScanLayout::GetSubscanAngleByIndex(const size_t i) const
{
  return this->subscanLayout.GetAngle(i);
}

double MultiLayerLaserScanLayout::GetSubscanAngle(const size_t i) const
{
  if (i >= this->length)
    throw std::out_of_range("Requested point is outside of the current layout.");

  return this->GetSubscanAngleByIndex(this->GetSubscanIndex(i));
}

ros::Duration MultiLayerLaserScanLayout::GetTimeByIndex(
    const size_t scanIndex, const size_t subscanIndex) const
{
  return this->scanLayout.GetTime(scanIndex) + this->subscanLayout.GetTime(subscanIndex);
}

ros::Duration MultiLayerLaserScanLayout::GetTime(size_t i) const
{
  if (i >= this->length)
    throw std::out_of_range("Requested point is outside of the current layout.");

  return this->GetTimeByIndex(this->GetScanIndex(i), this->GetSubscanIndex(i));
}

size_t MultiLayerLaserScanLayout::Length() const
{
  return this->length;
}

size_t MultiLayerLaserScanLayout::GetScanIndex(size_t i) const
{
  return i / this->subscanLength;
}

size_t MultiLayerLaserScanLayout::GetSubscanIndex(size_t i) const
{
  return i % this->subscanLength;
}

void MultiLayerLaserScanLayout::GetAll(size_t i, double& _scanAngle,
  double& _subscanAngle, ros::Duration& _time) const
{
  if (i >= this->length)
    throw std::out_of_range("Requested point is outside of the current layout.");

  const auto subscanIndex = this->GetSubscanIndex(i);
  const auto scanIndex = this->GetScanIndex(i);

  _subscanAngle = this->GetSubscanAngleByIndex(subscanIndex);
  _scanAngle = this->GetScanAngleByIndex(scanIndex, subscanIndex);
  _time = this->GetTimeByIndex(scanIndex, subscanIndex);
}

void MultiLayerLaserScanLayout::FillMsg(MultiLayerLaserScan &msg) const
{
  this->scanLayout.FillMsg(msg.scan_layout);
  this->subscanLayout.FillMsg(msg.subscan_layout);
  this->scanAngularVelocity->FillMsg(msg.scan_offsets_during_subscan);
}

}