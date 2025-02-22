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

#include <cmath>

#include "sensor.hpp"

Sensor::Sensor(std::vector<Line> walls, double x, double y, double yaw) noexcept:
  m_frame{},
  m_frameMutex{},
  m_walls{walls},
  m_x0{x},
  m_y0{y},
  m_x1{},
  m_y1{}
{
  double const sensorMax = 4.0;
  m_x1 = x + std::cos(yaw) * sensorMax;
  m_y1 = y + std::sin(yaw) * sensorMax;
}

void Sensor::setFrame(opendlv::sim::Frame const &frame) noexcept
{
  std::lock_guard<std::mutex> lock(m_frameMutex);
  m_frame = frame;
}

std::pair<opendlv::proxy::VoltageReading, opendlv::proxy::DistanceReading> Sensor::step() noexcept
{
  opendlv::sim::Frame frame;
  {
    std::lock_guard<std::mutex> lock(m_frameMutex);
    frame = m_frame;
  }

  double xf{frame.x()};
  double yf{frame.y()};
  double yawf{frame.yaw()};

  double x1{xf + m_x0 * std::cos(yawf) - m_y0 * std::sin(yawf)};
  double y1{yf + m_x0 * std::sin(yawf) + m_y0 * std::cos(yawf)};
  
  double x2{xf + m_x1 * std::cos(yawf) - m_y1 * std::sin(yawf)};
  double y2{yf + m_x1 * std::sin(yawf) + m_y1 * std::cos(yawf)};

  Line line{x1, y1, x2, y2};

  float voltage = 0.0f;
  // float distance = 0.34f;
  float distance = 4.0f;
  for (Line wall : m_walls) {
    std::pair<bool, double> intersection = checkIntersectionAndDistance(line, wall);
    if (intersection.first) {
      if (intersection.second < 0.0) {
        voltage = 1.8f;
        distance = 0.0f; 
      } else if (intersection.second < 4.0f) {
        voltage = convertDistanceToIrVoltage(distance);
        distance = static_cast<float>(intersection.second);
      }
    }
  }

  opendlv::proxy::VoltageReading voltageReading;
  voltageReading.voltage(voltage);

  opendlv::proxy::DistanceReading distanceReading;
  distanceReading.distance(distance);
  
  return std::pair<opendlv::proxy::VoltageReading,
         opendlv::proxy::DistanceReading>(voltageReading, distanceReading);
}
  
std::pair<bool, double> Sensor::checkIntersectionAndDistance(Line a, Line b) const noexcept
{
  double s1_x{a.x2() - a.x1()};
  double s1_y{a.y2() - a.y1()};
  double s2_x{b.x2() - b.x1()};
  double s2_y{b.y2() - b.y1()};

  double s{(-s1_y * (a.x1() - b.x1()) + s1_x * (a.y1() - b.y1())) / (-s2_x * s1_y + s1_x * s2_y)};
  double t{(s2_x * (a.y1() - b.y1()) - s2_y * (a.x1() - b.x1())) / (-s2_x * s1_y + s1_x * s2_y)};

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    double ix = a.x1() + (t * s1_x);
    double iy = a.y1() + (t * s1_y);
    double distance = sqrt((ix - a.x1()) * (ix - a.x1()) + (iy - a.y1()) * (iy - a.y1()));
    return std::pair<bool, double>{true, distance};
  }

  return std::pair<bool, double>{false, 0.0};
}

float Sensor::convertDistanceToIrVoltage(double distance) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  // double sensorVoltage = 3.4 - distance * 9.9;
  double sensorVoltage = 4.0*10 - distance * 9.9;
  double voltage = sensorVoltage * voltageDividerR2 / (voltageDividerR1 + voltageDividerR2);

  return static_cast<float>(voltage);
}
