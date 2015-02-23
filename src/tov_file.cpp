/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#include "kvh1750/tov_file.h"

#include <termios.h>

#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <byteswap.h>

namespace kvh
{

/**
 * Default constructor.
 * \param[in] addr File location to read data from
 * \param[in] tm Number of milliseconds to wait. -1 blocks, 0 is nonblock
 */
TOVFile::TOVFile(const std::string& addr, int tm, const std::string& tov_addr)
  :IOModule(),
  _fd(-1),
  _tov_fd(-1),
  _efd(-1),
  _tov_efd(-1),
  _timeout(tm),
  _events(),
  _tm(),
  _valid_tm(false)
{
  if(addr.empty())
  {
    throw std::invalid_argument("Empty address specified");
  }

  //right now we only read, but there needs to be a write interface
  _fd = open(addr.c_str(), O_RDWR);

  if(_fd < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  if(!tov_addr.empty())
  {
    _tov_fd = open(tov_addr.c_str(), O_RDONLY);
  }

  _efd = epoll_create(1);

  if(_efd < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  //note: first element of events is being used for control
  //to reduce memory usage slightly
  _events[0].data.fd = _fd;
  _events[0].events = EPOLLIN;
  int res = epoll_ctl(_efd, EPOLL_CTL_ADD, _fd, &_events[0]);

  if(res < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  _tov_efd = epoll_create(1);

  if(_tov_efd < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  //If TOV is disabled, both epolls wait on the same FD
  //to maintain invariants
  int dummy_fd = (tov_addr.empty() ? _fd : _tov_fd);
  //note: first element of events is being used for control
  //to reduce memory usage slightly
  _events[0].data.fd = dummy_fd;
  _events[0].events = EPOLLIN;// | EPOLLET;
  res = epoll_ctl(_tov_efd, EPOLL_CTL_ADD, dummy_fd, &_events[0]);

  if(res < 0)
  {
    throw std::runtime_error(strerror(errno));
  }
}

/**
 * Default destructor
 */
TOVFile::~TOVFile()
{
  if(_fd >= 0)
  {
    close(_fd);
  }

  if(_tov_fd >= 0)
  {
    close(_tov_fd);
  }
}

/**
 * Low level read interface for A POSIX style file descriptor.
 * \param[in,out] buff Buffer to store data
 * \param[in,out] bytes Number of bytes read
 * \param[out] Flag indicating if read was successful
 */
bool TOVFile::read(char* buff, size_t max_bytes, size_t& bytes, bool tov)
{
  ;
  int num_events = 0;
  //wait on TOV if available and desired, direct input otherwise
  int wait_fd = (_valid_tm || !tov ? _efd : _tov_efd);
  num_events = epoll_wait(wait_fd, &_events[0], TOVFile::NumEvents, _timeout);
  //grab time immediately in case we need it
  duration_t wait = std::chrono::high_resolution_clock::now().time_since_epoch();

  if(num_events == 0)
  {
    return false;
  }

  //only update time if at start of new message
  if(!_valid_tm)
  {
    _tm = wait;
    _valid_tm = true;
  }

  for(size_t ii = 0; ii < TOVFile::NumEvents; ++ii)
  {
    if(((_events[ii].events & EPOLLERR) ||
       (_events[ii].events & EPOLLHUP) || (!(_events[ii].events & EPOLLIN))))
    {
      return false;
    }
    //always read actual imu FD
    int rc = ::read(_fd, buff, max_bytes);

    if(rc < 0)
    {
      return false;
    }

    bytes = static_cast<size_t>(rc);
  }

  return true;
}

/**
 * Writes data to the device.
 * \param[out] Flag indicating if bytes are written
 */
bool TOVFile::write(const char* buff, size_t bytes)
{
  ssize_t res = ::write(_fd, buff, bytes);
  return (res > 0);
}

/**
 * Flush buffers to clear up any cached data
 */
void TOVFile::flush_buffers()
{
  tcflush(_fd, TCIOFLUSH);
}

/**
 * Reports time when current message was read.
 * \param[in,out] secs Seconds since epoch
 * \param[in,out] nsecs Nanoseconds since the Seconds field
 */
void TOVFile::time(uint64_t& secs, uint64_t& nsecs)
{
  duration_t d_secs = std::chrono::duration_cast<std::chrono::seconds>(_tm);
  duration_t diff_t = _tm - d_secs;
  duration_t d_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(diff_t);
  secs = d_secs.count();
  nsecs = d_nsecs.count();
}

/**
 * Resets time to set for next read.
 */
void TOVFile::reset_time()
{
  _valid_tm = false;
  _tm = std::chrono::high_resolution_clock::duration::zero();
}

}
