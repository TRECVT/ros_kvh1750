/**
 * Unit testing KVH Interfaces.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/19/2015
 */
#include "kvh1750/imu.h"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

namespace trec
{

/**
 * Stub class for unit testing data
 */
class KVHStub : public trec::KVHBase
{
public:
  KVHStub();
  virtual ~KVHStub() = default;

protected:
  size_t read(std::vector<char>& buff, uint64_t& secs,
              uint64_t& nsecs, bool& is_c, bool& is_da);
protected:
  std::vector<std::vector<char> > _responses;
  size_t _counter;
};

/**
 * Default constructor
 */
KVHStub::KVHStub() :
  _responses(),
  _counter(0)
{
  //accelerometers only
  std::vector<char> response(sizeof(kvh::RawMessage), 0);
  std::array<char, sizeof(kvh::RawMessage)> valid =
    kvh::byte_array(0xFE, 0x81, 0xFF, 0x55, 0x3B, 0x04, 0x1F, 0x78, 0xB9, 0xB9,
                    0x66, 0x13, 0xBA, 0x23, 0x4B, 0x38, 0xBB, 0x7C, 0x04, 0x10,
                    0xB8, 0xDE, 0xF4, 0x80, 0x3F, 0x7F, 0x4B, 0x7B, 0x70, 0x1F,
                    0x00, 0x1D, 0xF1, 0x2F, 0xB6, 0xEE
                   );
  response.assign(valid.begin(), valid.end());
  _responses.push_back(response);
  //fully valid
  valid = kvh::byte_array(0xFE, 0x81, 0xFF, 0x55, 0xB3, 0xD0, 0xBB, 0xE7, 0x32,
                          0x19, 0x3E, 0xF4, 0x30, 0xAC, 0x19, 0xA5, 0xBA, 0x8F, 0x78, 0xD6, 0x3A, 0x29,
                          0x05, 0x94, 0x3F, 0x7E, 0xCB, 0xD4, 0x77, 0x68, 0x00, 0x1D, 0xAD, 0xC5, 0xF6,
                          0x51);
  response.assign(valid.begin(), valid.end());
  _responses.push_back(response);
  //bad checksum
  valid = kvh::byte_array(0xFE, 0x81, 0xFF, 0x55, 0xB4, 0x6B, 0x86, 0xC6, 0xB4,
                          0x4F, 0x5B, 0x83, 0xB4, 0x10, 0x6D, 0x09, 0xBB, 0x71, 0x3C, 0xB7, 0xBB,
                          0xAB, 0xD9, 0x14, 0x3F, 0x7E, 0x31, 0x9A, 0x77, 0x69, 0x00, 0x1D, 0x95,
                          0xBB, 0xDB, 0x4F);
  response.assign(valid.begin(), valid.end());
  _responses.push_back(response);
}

/**
 * Simplified read which draws from canned data.
 */
size_t KVHStub::read(std::vector<char>& buff, uint64_t& secs,
                     uint64_t& nsecs, bool& is_c, bool& is_da)
{
  is_c = true;
  is_da = true;
  if(_counter < _responses.size())
  {
    typedef std::chrono::high_resolution_clock::duration duration_t;

    buff = _responses[_counter++];
    duration_t tm;
    tm = std::chrono::high_resolution_clock::now().time_since_epoch();
    duration_t d_secs = std::chrono::duration_cast<std::chrono::seconds>(tm);
    duration_t diff_t = tm - d_secs;
    duration_t d_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(diff_t);
    secs = d_secs.count();
    nsecs = d_nsecs.count();
  }

  return (buff.size() == sizeof(kvh::RawMessage) ? VALID : PARTIAL_READ);
}

};

namespace
{
std::shared_ptr<trec::KVHBase> stub(new trec::KVHStub());
}

TEST(TestSuite, earlyMessageTest)
{
  //Need to exploit polymorphism
  kvh::Message msg;
  EXPECT_EQ(trec::VALID, stub->read(msg));
  EXPECT_EQ(true, msg.valid());
  EXPECT_EQ(false, msg.valid_gyro_x());
  EXPECT_EQ(false, msg.valid_gyro_y());
  EXPECT_EQ(false, msg.valid_gyro_z());
  EXPECT_EQ(true, msg.valid_accel_x());
  EXPECT_EQ(true, msg.valid_accel_y());
  EXPECT_EQ(true, msg.valid_accel_z());
  EXPECT_EQ(31, msg.sequence_number());
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
  //Test that C->C conversion is idempotent
  msg.to_celsius();
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
  //test conversion to farenheit
  msg.to_farenheit();
  EXPECT_EQ(false, msg.is_celsius());
  EXPECT_EQ(84, msg.temp());
}

TEST(TestSuite, validMessageTest)
{
  kvh::Message msg;
  EXPECT_EQ(trec::VALID, stub->read(msg));
  EXPECT_EQ(true, msg.valid());
  EXPECT_EQ(true, msg.valid_gyro_x());
  EXPECT_EQ(true, msg.valid_gyro_y());
  EXPECT_EQ(true, msg.valid_gyro_z());
  EXPECT_EQ(true, msg.valid_accel_x());
  EXPECT_EQ(true, msg.valid_accel_y());
  EXPECT_EQ(true, msg.valid_accel_z());
  EXPECT_EQ(104, msg.sequence_number());
  EXPECT_EQ(29, msg.temp());
  EXPECT_EQ(true, msg.is_celsius());
}

TEST(TestSuite, invalidMessageTest)
{
  kvh::Message msg;
  EXPECT_EQ(trec::BAD_CRC, stub->read(msg));
  EXPECT_EQ(false, msg.valid());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}