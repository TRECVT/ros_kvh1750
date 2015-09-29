/**
 * Plugin interface for adding custom processing for KVH messages.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 09/29/2015
 */
#ifndef _KVH_PLUGIN_H_
#define _KVH_PLUGIN_H_

#include "kvh1750/types.h"

namespace kvh
{

class KVHMessageProcessorBase
{
public:
  virtual ~KVHMessageProcessorBase() {};

  virtual void process_message(const kvh::Message& msg) = 0;
protected:
  KVHMessageProcessorBase() {};
};

}

#endif
