/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#include "kvh1750/imu.h"

#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <byteswap.h>

namespace trec
{

/**
 * Default constructor
 */
KVHBase::KVHBase()
  :_buff(sizeof(kvh::RawMessage), 0),
  _bytes_read(0)
{
}

/**
 * Public read interface, which attempts to read a new KVH message
 * \param[in] msg Storage location for new message.
 * \param[out] size_t Flag indicating read success.
 */
size_t KVHBase::read(kvh::Message& msg)
{
  uint64_t secs = 0;
  uint64_t nsecs = 0;
  bool is_c = false;
  bool is_da = false;
  size_t result = this->read(_buff, secs, nsecs, is_c, is_da);

  std::vector<char>::iterator match = _buff.end();
  bool header_start = find_header(match);

  if(header_start && result == VALID)
  {
    kvh::RawMessage* raw = reinterpret_cast<kvh::RawMessage*>(&_buff.front());
    result = (msg.from_raw(*raw, secs, nsecs, is_c, is_da) ? VALID : BAD_CRC);
    reset_buffer();
  }
  else
  {
    reset_partial_buffer(match);
    result = PARTIAL_READ;
  }

  return result;
}

/**
 * Function to search the memory buffer for the header sequence.
 * \param[in,out] iterator pointing to location of header
 * \param[out] Flag indicating if header is at the start of the header.
 */
bool KVHBase::find_header(std::vector<char>::iterator& match)
{
  std::vector<char>::iterator buffer_end = _buff.begin() + _bytes_read;
  match = std::search(_buff.begin(), buffer_end, kvh::Header.begin(),
    kvh::Header.end());

  return match == _buff.begin();
}

/**
 * Clears and resets the memory buffer used to store the raw message being
 * processed.
 */
void KVHBase::reset_buffer()
{
  _buff.clear();
  _buff.resize(sizeof(kvh::RawMessage), 0);
  _bytes_read = 0;
}

/**
 * Stores a partially read message at the start of the buffer
 * to simplify processing.
 * \param[in] match Iterator pointing to start of header.
 */
void KVHBase::reset_partial_buffer(const std::vector<char>::iterator& match)
{
  std::vector<char> new_buffer;
  std::vector<char>::iterator buffer_end = _buff.begin() + _bytes_read;
  //start at either the match, or the longest missable substring
  std::vector<char>::iterator start = std::min(match, std::max(_buff.begin(),
    buffer_end - (sizeof(kvh::Header) - 1)));
  new_buffer.insert(new_buffer.begin(), start, buffer_end);
  new_buffer.resize(sizeof(kvh::RawMessage), 0);
  _bytes_read = buffer_end - start;
  _buff.swap(new_buffer);
}

/**
 * Number of bytes available for writing in the buffer.
 */
size_t KVHBase::bytes_remaining() const
{
  return sizeof(kvh::RawMessage) - _bytes_read;
}

/**
 * Default constructor.
 * \param[in] addr File location to read data from
 * \param[in] tm Number of milliseconds to wait. -1 blocks, 0 is nonblock
 */
KVH1750::KVH1750(const std::string& addr, int tm) :
  KVHBase(),
  _fd(-1),
  _efd(-1),
  _timeout(tm),
  _events(),
  _tm()
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

  _efd = epoll_create(1);

  if(_efd < 0)
  {
    throw std::runtime_error(strerror(errno));
  }

  //note: first element of events is being used for control
  //to reduce memory usage slightly
  _events[0].data.fd = _fd;
  _events[0].events = EPOLLIN;
  //TODO: We should be creating a file-descriptor here for the timing element,
  // not the reading element. Then we can grab the time correctly, and block on
  // the read.
  int res = epoll_ctl(_efd, EPOLL_CTL_ADD, _fd, &_events[0]);

  if(res < 0)
  {
    throw std::runtime_error(strerror(errno));
  }
}

/**
 * Default destructor
 */
KVH1750::~KVH1750()
{
  if(_fd >= 0)
  {
    close(_fd);
  }
}

/**
 * Low level read interface for A POSIX style file descriptor.
 * \param[in,out] buff Buffer to store data
 * \param[in,out] secs Timestamp, seconds since epoch.
 * \param[in,out] nsecs Timestamp, nanoseconds since the seconds since epoch.
 * \param[in,out] is_c Flag indicating if temperature is in Celsius
 * \param[in,out] is_da Flag indicating if the angular velocities are raw or filtered.
 */
size_t KVH1750::read(std::vector<char>& buff, uint64_t& secs, uint64_t& nsecs, bool& is_c, bool& is_da)
{
  //TODO: Track state of KVH
  is_da = true;
  is_c = true;
  int num_events = epoll_wait(_efd, &_events[0], KVH1750::NumEvents, _timeout);
  //only update time if at start of new message
  if(_bytes_read == 0)
  {
    _tm = std::chrono::high_resolution_clock::now().time_since_epoch();
  }
  if(num_events == 0)
  {
    return BAD_READ;
  }

  for(size_t ii = 0; ii < KVH1750::NumEvents; ++ii)
  {
    if((_events[ii].events & EPOLLERR) ||
       (_events[ii].events & EPOLLHUP) || (!(_events[ii].events & EPOLLIN)))
    {
      return FATAL_ERROR;
    }
    int rc = ::read(_fd, &_buff[_bytes_read], bytes_remaining());

    if(rc < 0)
    {
      return BAD_READ;
    }

    _bytes_read += rc;
  }

  size_t result = (_bytes_read == sizeof(kvh::RawMessage) ? VALID : PARTIAL_READ);
  if(result)
  {
    result = VALID;
    duration_t d_secs = std::chrono::duration_cast<std::chrono::seconds>(_tm);
    duration_t diff_t = _tm - d_secs;
    duration_t d_nsecs = std::chrono::duration_cast<std::chrono::nanoseconds>(diff_t);
    secs = d_secs.count();
    nsecs = d_nsecs.count();
  }

  return result;
}

}
