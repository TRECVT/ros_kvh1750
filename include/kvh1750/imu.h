/**
 * Interfacing with KVH 1750 over RSS-422.
 * \author Jason Ziglar <jpz@vt.edu>, based on code form Eric L. Hahn <erichahn@vt.edu>
 * \date 02/15/2014
 * Copyright 2014, Virginia Tech. All Rights Reserved.
 */
#ifndef __KVH_1750_IMU_h__
#define __KVH_1750_IMU_h__

#include "kvh1750/types.h"

#include <sys/epoll.h>

#include <chrono>

#include <vector>

namespace trec
{

/**
 * Reasons why a processing attempt might fail
 */
enum ParseResults
{
  VALID = 0,
  BAD_SIZE,
  BAD_READ,
  FATAL_ERROR,
  BAD_CRC,
  PARTIAL_READ,
  OVER_TEMP
};

class KVHBase
{
public:
  KVHBase();
  virtual ~KVHBase() =default;

  size_t read(kvh::Message& msg);
  //TODO: Add configuration commands
protected:
  virtual size_t read(std::vector<char>& buff, uint64_t& secs,
    uint64_t& nsecs, bool& is_c, bool& is_da) = 0;

  bool find_header(std::vector<char>::iterator& match);
  void reset_buffer();
  void reset_partial_buffer(const std::vector<char>::iterator& match);
  size_t bytes_remaining() const;
protected:
  std::vector<char> _buff;
  size_t _bytes_read;
};

/**
 * Interface to a KVH1750 over RS-485
 */
class KVH1750 : public KVHBase
{
public:
  static const int DefaultTimeout = -1; //Blocking
  static const size_t NumEvents = 1;
public:
  KVH1750(const std::string& addr = "/dev/ttyS4", int tm = DefaultTimeout);
  virtual ~KVH1750();

protected:
  virtual size_t read(std::vector<char>& buff, uint64_t& secs,
    uint64_t& nsecs, bool& is_c, bool& is_da);

protected:
  typedef std::chrono::high_resolution_clock::duration duration_t;
  int _fd;
  int _efd;
  int _timeout;
  epoll_event _events[NumEvents];
  duration_t _tm;
};

};

#endif