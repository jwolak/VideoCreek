/*
 * CameraHandler.cpp
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

#include "CameraHandler.h"
#include "EquinoxLogger.h"

video_creek::CameraHandler::~CameraHandler()
{
  if(nullptr != mFramesGrabberThread_)
  {
    mFramesGrabberThread_->join();
  }
}

bool video_creek::CameraHandler::openCam()
{
  equinox::trace("%s","[CameraHandler] Camera device is being opening...");

  return true;
}

bool video_creek::CameraHandler::start(std::function<void(void)> frameReceivedCallback)
{

  equinox::trace("%s","[CameraHandler] CameraHandler is starting...");

  if (frameReceivedCallback != nullptr)
  {
    mFrameReceivedCallback_ = frameReceivedCallback;
    equinox::debug("%s", "[CameraHandler] Frame received callback is set");
  }
  else
  {
    equinox::error("%s", "[CameraHandler] Frame received callback is null");
    return false;
  }

  if(nullptr == (mFramesGrabberThread_ = std::make_shared<std::thread>(&CameraHandler::runCamera, this)))
  {
    equinox::critical("%s", "[CameraHandler] Camera handler thread start failed");
    return false;
  }

  equinox::critical("%s", "[CameraHandler] Camera handler thread start successful");
  return true;
}

void video_creek::CameraHandler::runCamera()
{
  equinox::trace("%s","[CameraHandler] CameraHandler thread is starting...");

  while(true)
  {
    std::unique_lock<std::mutex> lock(mFramesGrabberThreadMutex_);

    equinox::trace("%s", "[CameraHandler] Camera handler thread is waiting for signal...");
    mConditionVariableFramesGrabberThread_.wait(lock, [this]()
    {
      return mNewFrameRequestedFlag_ == true;
    });

    if (mNewFrameRequestedFlag_ == true)
    {
      equinox::trace("%s", "[CameraHandler] New frame request signal received");
      mNewFrameRequestedFlag_ = false;
      //get frame from cam
    }
  }
}

void video_creek::CameraHandler::requestNewFrame()
{
  equinox::trace("%s", "[CameraHandler] New frame request received");
  mNewFrameRequestedFlag_ = true;
}
