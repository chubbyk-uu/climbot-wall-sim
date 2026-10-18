// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "climbot_gazebo/vacuum_io.hpp"
#include "climbot_gazebo/vacuum_state.hpp"
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace climbot_gazebo::vacuum
{
namespace
{
gz::msgs::FluidPressure message(double time, double pressure, double variance)
{
  gz::msgs::FluidPressure out;
  auto ns = static_cast<int64_t>(std::llround(time * 1e9));
  out.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
  out.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
  auto data = out.mutable_header()->add_data();data->set_key("frame_id");
  data->add_value("chassis_link");out.set_pressure(pressure);out.set_variance(variance);
  return out;
}
}
VacuumIO::VacuumIO(const std::shared_ptr<sdf::Element> & sdf)
{
  auto io = sdf->GetElement("io");auto s = io->GetElement("sensor");
  SensorParameters p;
  p.rate_hz = s->Get<double>("rate_hz");p.noise_pa = s->Get<double>("noise_pa");
  p.bias_pa = s->Get<double>("bias_pa");p.delay_s = s->Get<double>("delay_s");
  p.minimum_pa = s->Get<double>("minimum_pa");p.maximum_pa = s->Get<double>("maximum_pa");
  p.seed = s->Get<unsigned>("seed");sensor_ = std::make_unique<PressureSensor>(p);
  double rate = io->Get<double>("truth_rate_hz");
  if (!std::isfinite(rate) || rate <= 0 || rate > 1000)
  {throw std::invalid_argument("invalid truth publication rate");}
  truthPeriod_ = 1 / rate;
  if (io->HasElement("opening")) {
    for (auto e = io->GetElement("opening"); e; e = e->GetNextElement("opening")) {
      double time = e->Get<double>("time_s"), area = e->Get<double>("area_m2");
      if (!std::isfinite(time) || !std::isfinite(area) || time < 0 || area < 0 || area > .02 ||
        (openings_.empty() ? time != 0 : time <= openings_.back().first))
      {throw std::invalid_argument("invalid independent opening schedule");}
      openings_.emplace_back(time, area);
    }
  }
  if (io->HasElement("event")) {
    for (auto e = io->GetElement("event"); e; e = e->GetNextElement("event")) {
      Command command{e->Get<double>("time_s"), e->Get<int>("kind"),
        e->Get<int>("index"), e->Get<int>("value")};
      if (!std::isfinite(command.time) || command.time < 0 || command.kind < 0 ||
        command.kind > 2 || command.index < 0 || command.index > 2 || command.value < 0 ||
        command.value > (command.kind == 2 ? 2 : 1) ||
        (command.kind == 2 && command.index != 0) ||
        (!events_.empty() && command.time < events_.back().time))
      {throw std::invalid_argument("invalid ordered vacuum event");}
      events_.push_back(command);
    }
  }
  pressure_ = node_.Advertise<gz::msgs::FluidPressure>("/suction/pressure");
  ambient_ = node_.Advertise<gz::msgs::FluidPressure>("/suction/ambient_pressure");
  truth_ = node_.Advertise<gz::msgs::StringMsg>("/suction/ground_truth/state");
  for (int i = 0; i < 3; ++i) {
    std::string topic = "/suction/fans/fan_" + std::to_string(i);
    std::function<void(const gz::msgs::Boolean &)> enable =
      [this, i](const auto & m) {enqueue(0, i, m.data());};
    std::function<void(const gz::msgs::Boolean &)> fail =
      [this, i](const auto & m) {enqueue(1, i, m.data());};
    if (!node_.Subscribe(topic + "/enabled", enable) ||
      !node_.Subscribe(topic + "/failed", fail))
    {throw std::runtime_error("fan command subscription failed");}
  }
  std::function<void(const gz::msgs::Int32 &)> fault =
    [this](const auto & m) {enqueue(2, 0, m.data());};
  if (!node_.Subscribe("/suction/sensor/fault", fault))
  {throw std::runtime_error("sensor fault subscription failed");}
  reset();
}
void VacuumIO::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();nextEvent_ = 0;revision_ = 0;rejected_ = 0;
  openingArea_ = openings_.empty() ? 0 : openings_.front().second;
  enabled_ = {true, true, true};failed_ = {};sensor_->reset();last_.reset();nextTruth_ = 0;
}
void VacuumIO::enqueue(int kind, int index, int value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if ((kind == 2 && (value < 0 || value > 2)) || pending_.size() >= 1024) {
    ++rejected_;return;
  }
  pending_.push_back({0, kind, index, value});
}
void VacuumIO::execute(const Command & c)
{
  if (c.kind == 0) {enabled_.at(c.index) = c.value;} else if (c.kind == 1) {
    failed_.at(c.index) = c.value;
  } else {sensor_->fault(c.value);}
  ++revision_;
}
std::vector<double> VacuumIO::apply(double time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!openings_.empty()) {
    auto next = std::upper_bound(openings_.begin(), openings_.end(), time,
        [](double t, const auto & point) {return t < point.first;});
    if (next == openings_.begin()) {
      openingArea_ = next->second;
    } else if (next == openings_.end()) {
      openingArea_ = openings_.back().second;
    } else {
      const auto & previous = *(next - 1);
      double fraction = (time - previous.first) / (next->first - previous.first);
      openingArea_ = previous.second + fraction * (next->second - previous.second);
    }
  }
  while (nextEvent_ < events_.size() && events_[nextEvent_].time <= time + 1e-10) {
    execute(events_[nextEvent_++]);
  }
  for (const auto & c : pending_) {
    execute(c);
  }
  pending_.clear();
  return {enabled_[0] && !failed_[0] ? 1.0 : 0.0,
    enabled_[1] && !failed_[1] ? 1.0 : 0.0, enabled_[2] && !failed_[2] ? 1.0 : 0.0};
}
void VacuumIO::publish(double time, std::vector<double> & state)
{
  // A terminal domain failure invalidates the chamber solution. Keep the
  // explicit fault observable without stamping stale pressure as a fresh
  // sensor measurement. Reset is required before sampling resumes.
  if (state[RuntimeFailed] == 0) {
    auto sample = sensor_->update(time, state[CavityPressure]);
    if (sample) {
      last_ = sample;pressure_.Publish(message(sample->stamp_s, sample->pressure_pa,
          sample->variance_pa2));
    }
  }
  for (int i = 0; i < 3; ++i) {
    state[Enabled0 + i] = enabled_[i];state[Failed0 + i] = failed_[i];
  }
  state[SensorFault] = sensor_->fault();state[CommandRevision] = revision_;
  {std::lock_guard<std::mutex> lock(mutex_);state[RejectedCommands] = rejected_;}
  state[SensorSequence] = sensor_->samples();
  state[SensorStamp] = last_ ? last_->stamp_s : -1;
  state[SensorDelivery] = last_ ? last_->delivery_s : -1;
  state[SensorValue] = last_ ? last_->pressure_pa : 0;
  state[SensorVariance] = last_ ? last_->variance_pa2 : 0;
  state[SensorSaturated] = last_ ? last_->saturated : false;
  if (time + 1e-10 < nextTruth_) {return;}
  nextTruth_ += truthPeriod_;
  ambient_.Publish(message(time, state[AmbientPressure], 0));
  std::ostringstream json;json << std::setprecision(17) << '{';
  for (std::size_t i = 0; i < Count; ++i) {
    if (i) {json << ',';}
    json << '\"' << kNames[i] << "\":" << state[i];
  }
  json << '}';gz::msgs::StringMsg truth;truth.set_data(json.str());truth_.Publish(truth);
}
}  // namespace climbot_gazebo::vacuum
