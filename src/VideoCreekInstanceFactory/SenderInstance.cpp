/*
 * SenderInstance.cpp
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

#include "SenderInstance.h"
#include "EquinoxLogger.h"

video_creek::SenderInstance::~SenderInstance()
{
  if(nullptr != mFrameSenderThread_)
  {
    mFrameSenderThread_->join();
  }
}

void video_creek::SenderInstance::newFrameProducedCallback()
{
  mNewFrameProducedFlag_ = true;
  mConditionVariableFramesSenderThread_.notify_all();
}

void video_creek::SenderInstance::compressedFrameIsReadyCallback()
{
  mCompressedFrameIsReadyFlag_ = true;
  mConditionVariableFramesSenderThread_.notify_all();
}

void video_creek::SenderInstance::compressedFrameIsSentInfoCallback()
{
  mInfoPacketIsSentFlag_ = true;
  mConditionVariableFramesSenderThread_.notify_all();
}

void video_creek::SenderInstance::stop()
{
  equinox::trace("%s", "[SenderInstance] CameraHandler requested to be stopped");
  mCameraHandler_->stop();

  equinox::trace("%s", "[SenderInstance] CompressionHandler requested to be stopped");
  mCompressionHandler_->stop();

  equinox::trace("%s", "[SenderInstance] UdpStreamer requested to be stopped");
  mUdpStreamer_->stop();

  equinox::trace("%s", "[SenderInstance] SenderInstance thread requested to be stopped");
  mContinueLoop_ = false;
  mConditionVariableFramesSenderThread_.notify_all();

  equinox::trace("%s", "[SenderInstance] Waiting until SenderInstance is stopped...");
  if(nullptr != mFrameSenderThread_)
  {
    mFrameSenderThread_->join();
  }

  equinox::trace("%s", "[SenderInstance] SenderInstance is stopped");
}

bool video_creek::SenderInstance::start()
{
  if(!mCameraHandler_->openCam())
  {
    equinox::error("%s", "[SenderInstance] Open camera device failed");
    return false;
  }
  equinox::trace("%s", "[SenderInstance] Open camera device successful");

  if(!mCameraHandler_->start(std::bind(&video_creek::SenderInstance::newFrameProducedCallback, this)))
  {
    equinox::error("%s", "[SenderInstance] Start CameraHandler failed");
    return false;
  }
  equinox::trace("%s", "[SenderInstance] Start CameraHandler successful");

  if(!mUdpStreamer_->start(std::bind(&video_creek::SenderInstance::compressedFrameIsSentInfoCallback, this)))
  {
    equinox::error("%s", "[SenderInstance] Setup UDP streamer failed");
    return false;
  }
  equinox::trace("%s", "[SenderInstance] Setup UDP streamer successful");

  if(!mCompressionHandler_->start(std::bind(&video_creek::SenderInstance::compressedFrameIsReadyCallback, this)))
  {
    equinox::error("%s", "[SenderInstance] Start CompressionHandler failed");
    return false;
  }
  equinox::trace("%s", "[SenderInstance] Start CompressionHandler successful");

  if(nullptr == (mFrameSenderThread_ = std::make_shared<std::thread>(&SenderInstance::runSender, this)))
  {
    equinox::error("%s", "[SenderInstance] Launch Sender thread failed");
    return false;
  }
  equinox::trace("%s", "[SenderInstance] Launch Sender thread successful");

  equinox::trace("%s", "[SenderInstance] Sender start successful");
  return true;
}

void video_creek::SenderInstance::runSender()
{
  //TODO Init request ???
  mCameraHandler_->requestNewFrame();

  while(mContinueLoop_)
  {
    std::unique_lock<std::mutex> lock(mFramesSenderThreadMutex_);

    equinox::trace("%s", "[SenderInstance] SenderInstance thread is waiting for signal...");
    mConditionVariableFramesSenderThread_.wait(lock, [this]()
    {
      return ((mNewFrameProducedFlag_ == true) or (mCompressedFrameIsReadyFlag_ == true) or (mInfoPacketIsSentFlag_ == true) or (mContinueLoop_ == false));
    });

    if (mContinueLoop_ == false)
    {
      equinox::trace("%s", "[SenderInstance] SenderInstance thread is being stopped...");
      break;
    }

    if (mNewFrameProducedFlag_ == true)
    {
      mNewFrameProducedFlag_ = false;
      mCompressionHandler_->compressFrame();
    }

    if (mCompressedFrameIsReadyFlag_ == true)
    {
      mCompressedFrameIsReadyFlag_ = false;
      mUdpStreamer_->send();
    }

    if (mInfoPacketIsSentFlag_ == true)
    {
      mInfoPacketIsSentFlag_ == false;
      mCameraHandler_->requestNewFrame();
    }

  }

}
