#ifndef _STRUCT_HPP_
#define _STRUCT_HPP_

struct Pos
{
  double xl, xr;
  double y, z;
  bool servo;
  bool y_mode;

  // 重载 + 运算符
  Pos operator+(const Pos & other) const
  {
    return {xl + other.xl, xr + other.xr,        y + other.y,
            z + other.z,   servo || other.servo, y_mode || other.y_mode};
  }

  // 重载 - 运算符
  Pos operator-(const Pos & other) const
  {
    return {xl - other.xl, xr - other.xr,       y - other.y,
            z - other.z,   servo ^ other.servo, y_mode ^ other.y_mode};
  }

  // 重载 += 运算符
  Pos & operator+=(const Pos & other)
  {
    xl += other.xl;
    xr += other.xr;
    y += other.y;
    z += other.z;
    servo = servo || other.servo;
    y_mode = y_mode || other.y_mode;
    return *this;
  }

  // 重载 -= 运算符
  Pos & operator-=(const Pos & other)
  {
    xl -= other.xl;
    xr -= other.xr;
    y -= other.y;
    z -= other.z;
    servo = servo ^ other.servo;
    y_mode = y_mode ^ other.y_mode;
    return *this;
  }
};

#endif  // _STRUCT_HPP_