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

#ifndef LINE
#define LINE

class Line {
 public:
  Line(double, double, double, double) noexcept;
  ~Line() = default;

 public:
  double x1() const noexcept;
  double y1() const noexcept;
  double x2() const noexcept;
  double y2() const noexcept;

 private:
  double m_x1;
  double m_x2;
  double m_y1;
  double m_y2;

};

#endif
