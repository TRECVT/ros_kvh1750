/**
 * Basic types for KVH 1750 interfacing.
 * \author Jason Ziglar <jpz@vt.edu>
 * \date 02/16/2015
 */
#ifndef _KVH_1750_TYPES_H_
#define _KVH_1750_TYPES_H_

#include <array>

#include <string>
#include <stdint.h>

namespace kvh
{

//C++11 macro for casting ints to chars, to avoid narrowing
template <typename... A>
constexpr std::array<char, sizeof...(A)> byte_array(A... v)
{ return std::array<char, sizeof...(A)>{{static_cast<char>(v)...}}; }

//! Constant for convering accelerations to floating point values
const float Gravity = 9.80665;
const std::array<char, 4> Header = byte_array(0xFE, 0x81, 0xFF, 0x55);
const int16_t MaxTemp_C = 75;

typedef enum
{
  GYRO_X = 1,
  GYRO_Y = 1 << 1,
  GYRO_Z = 1 << 2,
  ACCEL_X = 1 << 4,
  ACCEL_Y = 1 << 5,
  ACCEL_Z = 1 << 6
} StatusBits;

namespace crc
{
const size_t Width = 32;
const uint64_t Poly = 0x04C11DB7;
const uint32_t XOr_In = 0xFFFFFFFF;
const uint32_t XOr_Out = 0x0;
const bool Reflect_In = false;
const bool Reflect_Out = false;
}

/**
 * Raw message format from KVH, straight off the wire
 */
struct RawMessage
{
  char Header[4];
  float rots[3];
  float accels[3];
  uint8_t status;
  uint8_t seq;
  int16_t temp;
  uint32_t crc;
};

/**
 * Usable form of KVH message. Importantly, data is in valid units.
 */
class Message
{
public:
  Message();
  Message(const RawMessage& raw, uint32_t secs, uint32_t nsecs,
    bool is_c, bool is_da);
  ~Message();

  bool from_raw(const RawMessage& raw, uint32_t secs, uint32_t nsecs,
    bool is_c, bool is_da);

  float gyro_x() const;
  float gyro_y() const;
  float gyro_z() const;
  float accel_x() const;
  float accel_y() const;
  float accel_z() const;

  int16_t temp() const;
  int16_t temp(bool& is_c) const;

  void time(uint32_t& secs, uint32_t& nsecs) const;
  uint8_t sequence_number() const;

  bool valid() const;
  bool valid_gyro_x() const;
  bool valid_gyro_y() const;
  bool valid_gyro_z() const;

  bool valid_accel_x() const;
  bool valid_accel_y() const;
  bool valid_accel_z() const;
  bool is_celsius() const;
  bool is_delta_angle() const;

  void to_celsius();
  void to_farenheit();
protected:
  typedef enum
  {
    X = 0,
    Y,
    Z,
    NUM_FIELDS
  } Indices;
  float _ang_vel[NUM_FIELDS];
  float _lin_accel[NUM_FIELDS];
  uint32_t _secs;
  uint32_t _nsecs;
  int16_t _temp;
  uint8_t _status;
  uint8_t _seq;
  bool _is_da;
  bool _is_c;
};

bool valid_checksum(const RawMessage& msg);
uint32_t compute_checksum(const char* buff, size_t len);

}

#endif