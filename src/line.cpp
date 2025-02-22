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

#include "line.hpp"

Line::Line(double x1, double y1, double x2, double y2) noexcept:
  m_x1(x1),
  m_x2(x2),
  m_y1(y1),
  m_y2(y2)
{
}

double Line::x1() const noexcept
{
  return m_x1;
}

double Line::x2() const noexcept
{
  return m_x2;
}

double Line::y1() const noexcept
{
  return m_y1;
}

double Line::y2() const noexcept
{
  return m_y2;
}
