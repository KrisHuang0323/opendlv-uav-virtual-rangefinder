/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SENSOR
#define SENSOR

#include <mutex>
#include <vector>

#include "opendlv-standard-message-set.hpp"

#include "line.hpp"

class Sensor {
 private:
  Sensor(Sensor const &) = delete;
  Sensor(Sensor &&) = delete;
  Sensor &operator=(Sensor const &) = delete;
  Sensor &operator=(Sensor &&) = delete;

 public:
  Sensor(std::vector<Line>, double, double, double) noexcept;
  ~Sensor() = default;

 public:
  void setFrame(opendlv::sim::Frame const &) noexcept;
  std::pair<opendlv::proxy::VoltageReading, opendlv::proxy::DistanceReading> 
    step() noexcept;

 private:
  std::pair<bool, double> checkIntersectionAndDistance(Line, Line) const noexcept;
  float convertDistanceToIrVoltage(double) const noexcept;

 private:
  opendlv::sim::Frame m_frame;
  std::mutex m_frameMutex;
  std::vector<Line> m_walls;
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
};

#endif
