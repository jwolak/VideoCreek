/*
 * UdpStreamer.cpp
 *
 *  Created on: 2023
 *      Author: Janusz Wolak
 */

/*-
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Janusz Wolak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include "UdpStreamer.h"
#include "EquinoxLogger.h"

video_creek::UdpStreamer::~UdpStreamer()
{
  if(nullptr != mSenderThread_)
  {
    mSenderThread_->join();
  }

  if(nullptr != mReceiverThread_)
  {
    mReceiverThread_->join();
  }

  if(nullptr != mUdpStreamerThread_)
  {
    mUdpStreamerThread_->join();
  }
}

void video_creek::UdpStreamer::stop()
{
  equinox::trace("%s", "[UdpStreamer] UdpStreamer thread requested to be stopped");
  mContinueLoop_ = false;
  mConditionVariableUdpStreamerThread_.notify_all();
  mConditionVariableSenderThread_.notify_all();

  equinox::trace("%s", "[UdpStreamer] Waiting until Sender thread is stopped");
  if(nullptr != mSenderThread_)
  {
    mSenderThread_->join();
  }
  equinox::trace("%s", "[UdpStreamer] Sender thread is stopped");

  equinox::trace("%s", "[UdpStreamer] Waiting until Receiver thread is stopped");
  if(nullptr != mReceiverThread_)
  {
    mReceiverThread_->join();
  }
  equinox::trace("%s", "[UdpStreamer] Receiver thread is stopped");

  equinox::trace("%s", "[UdpStreamer] Waiting until UdpStreamer thread is stopped");
  if(nullptr != mUdpStreamerThread_)
  {
    mUdpStreamerThread_->join();
  }
  equinox::trace("%s", "[UdpStreamer] Waiti UdpStreamer thread is stopped");

}

bool video_creek::UdpStreamer::start(std::function<void(void)> compressedFrameIsSentInfoCallback)
{

  if (nullptr != compressedFrameIsSentInfoCallback)
  {
    mCompressedFrameIsSentInfoCallback_ = compressedFrameIsSentInfoCallback;
  }
  else
  {
    return false;
  }

  if (nullptr == (mUdpStreamerThread_ = std::make_shared<std::thread>(&UdpStreamer::runUdpStreamer, this)))
  {
    equinox::error("%s", "[UdpStreamer] Launch UdpStreamer thread failed");
    return false;
  }
  equinox::trace("%s", "[UdpStreamer] Launch UdpStreamer thread successful");

  if (nullptr == (mReceiverThread_ = std::make_shared<std::thread>(&UdpStreamer::runReceiver, this)))
  {
    equinox::error("%s", "[UdpStreamer] Launch receiver thread failed");
    return false;
  }
  equinox::trace("%s", "[UdpStreamer] Launch receiver thread successful");

  if (nullptr == (mSenderThread_ = std::make_shared<std::thread>(&UdpStreamer::runSender, this)))
  {
    equinox::error("%s", "[UdpStreamer] Launch sender thread failed");
    return false;
  }
  equinox::trace("%s", "[UdpStreamer] Launch sender thread successful");

  return true;
}

void video_creek::UdpStreamer::send()
{
  mNewCompressFrameToBeSentFlag_ = true;
  mConditionVariableUdpStreamerThread_.notify_all();
}

void video_creek::UdpStreamer::runReceiver()
{
  while(mContinueLoop_)
  {
    //wait for frame
    //set flag when received


    //mNewFreameReceivedFlag_ = true;
    mConditionVariableUdpStreamerThread_.notify_all();

    //equinox::trace("%s", "[UdpStreamer] runReceiver looping...");
  }

  if (mContinueLoop_ == false)
  {
    equinox::trace("%s", "[UdpStreamer] Receiver thread is stopped");
  }
}

void video_creek::UdpStreamer::runSender()
{
  while(mContinueLoop_)
  {
    std::unique_lock<std::mutex> lock(mSenderThreadMutex_);

    equinox::trace("%s", "[UdpStreamer] Sender thread is waiting for signal...");
    mConditionVariableSenderThread_.wait(lock, [this]()
    {
      return ((mRequestSendFrameFlag_ == true) or (mContinueLoop_ == false));
    });

    if (mContinueLoop_ == false)
    {
      equinox::trace("%s", "[UdpStreamer] Sender thread is being stopped");
      break;
    }

    if (mRequestSendFrameFlag_ == true)
    {
      mRequestSendFrameFlag_ = false;
      equinox::trace("%s", "[UdpStreamer] Network handler is being sending the frame...");
      mNetworkHandler_->send();
      equinox::trace("%s", "[UdpStreamer] Network handler sent the frame");
      mNewFreameSentFlag_ = true;
      mConditionVariableUdpStreamerThread_.notify_all();
    }
  }
}

void video_creek::UdpStreamer::runUdpStreamer()
{
  while (mContinueLoop_)
  {
    std::unique_lock<std::mutex> lock(mUdpStreamerMutex_);

    equinox::trace("%s", "[UdpStreamer] UdpStreamer main thread is waiting for signal...");
    mConditionVariableUdpStreamerThread_.wait(lock, [this]()
    {
      return ((mNewCompressFrameToBeSentFlag_ == true) or (mNewFreameReceivedFlag_ == true) or (mNewFreameSentFlag_ == true) or (mContinueLoop_ == false));
    });

    if (mContinueLoop_ == false)
    {
      equinox::trace("%s", "[UdpStreamer] UdpStreamer thread is stopped");
      break;
    }

    if (mNewCompressFrameToBeSentFlag_ == true)
    {
      mNewCompressFrameToBeSentFlag_ = false;
      mRequestSendFrameFlag_ = true;
      mConditionVariableSenderThread_.notify_all();
    }

    if (mNewFreameSentFlag_ == true)
    {
      mNewFreameSentFlag_ = false;
      equinox::trace("%s", "[UdpStreamer] Frame is sent signal received...");
      mCompressedFrameIsSentInfoCallback_();
      equinox::trace("%s", "[UdpStreamer] Compressed frame is sent callback called");
    }

    if (mNewFreameReceivedFlag_ == true)
    {
      //decompress and inform
      mNewFreameReceivedFlag_ = false;
    }

  }
}
