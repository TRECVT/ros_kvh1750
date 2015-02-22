/**
 * Interfacing with KVH 1750 over RSS-422.
 * \author Jason Ziglar <jpz@vt.edu>, based on code form Eric L. Hahn <erichahn@vt.edu>
 * \date 02/15/2015
 * Copyright 2015, Virginia Tech. All Rights Reserved.
 */
#ifndef __KVH_1750_IMU_h__
#define __KVH_1750_IMU_h__

#include "imu.h"

#include <sys/epoll.h>

#include <chrono>

#include <vector>

namespace kvh
{

/**
 * Interface to a KVH1750 over RS-422, with support for the TOV signal
 * via a file descriptor.
 */
class TOVFile : public IOModule
{
public:
  static const int DefaultTimeout = -1; //Blocking
  static const size_t NumEvents = 1;
public:
  TOVFile(const std::string& addr = "/dev/ttyS4", int tm = DefaultTimeout,
    const std::string& tov_addr = "");
  virtual ~TOVFile();

  virtual bool read(char* buff, size_t max_bytes, size_t& bytes, bool tov);
  virtual bool write(const char* buff, size_t bytes);
  virtual void flush_buffers();
  virtual void time(uint64_t& secs, uint64_t& nsecs);
  virtual void reset_time();

protected:
  typedef std::chrono::high_resolution_clock::duration duration_t;
  int _fd; //! Actual data file descriptor
  int _tov_fd; //! TOV file descriptor
  int _efd; //! Actual data epoll file descriptor
  int _tov_efd; //! TOV epoll file descriptor
  int _timeout; //! Number of milliseconds to wait
  epoll_event _events[NumEvents]; //! epoll events to respond to
  duration_t _tm; //! Time when message was read
  bool _valid_tm; //! Flag indicating if time is valid
};

};

#endif