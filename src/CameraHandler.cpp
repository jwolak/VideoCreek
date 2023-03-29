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
  mVideoCapture_.release();

  if(nullptr != mFramesGrabberThread_)
  {
    mFramesGrabberThread_->join();
  }
}

bool video_creek::CameraHandler::openCam()
{
  equinox::trace("%s","[CameraHandler] Camera device is being opening...");

  if (!mVideoCapture_.open(mCmdArguments_->getCameraDeviceId()))
  {
    equinox::error("[CameraHandler] Failed to open camera device id: [%d]", mCmdArguments_->getCameraDeviceId());
    return false;
  }

  if (!mVideoCapture_.set(cv::CAP_PROP_FRAME_WIDTH, mCmdArguments_->getVideoWidth()))
  {
    equinox::debug("[CameraHandler] Camera frame width: [%d] failed", static_cast<int>(mCmdArguments_->getVideoWidth()));
  }
  equinox::debug("[CameraHandler] Camera frame width set: [%d]", static_cast<int>( mVideoCapture_.get(cv::CAP_PROP_FRAME_WIDTH)));


  if (!mVideoCapture_.set(cv::CAP_PROP_FRAME_HEIGHT , mCmdArguments_->getVideoHeight()))
  {
    equinox::debug("[CameraHandler] Camera frame width: [%d] failed", static_cast<int>(mCmdArguments_->getVideoHeight()));
  }
  equinox::debug("[CameraHandler] Camera frame height set: [%d]", static_cast<int>( mVideoCapture_.get(cv::CAP_PROP_FRAME_HEIGHT)));

  equinox::trace("[CameraHandler] Open camera device id: [%d] successful", mCmdArguments_->getCameraDeviceId());
  return true;
}

void video_creek::CameraHandler::stop()
{
  equinox::trace("%s", "[CameraHandler] CameraHandler thread requested to be stopped");
  mContinueLoop_ = false;
  mConditionVariableFramesGrabberThread_.notify_all();

  equinox::trace("%s", "[CameraHandler] Waiting until CameraHandler thread is stopped");
  if(nullptr != mFramesGrabberThread_)
  {
    mFramesGrabberThread_->join();
  }

  equinox::trace("%s", "[CameraHandler] CameraHandler thread is stopped");
}

bool video_creek::CameraHandler::start(std::function<void(void)> newFrameProducedCallback)
{

  equinox::trace("%s","[CameraHandler] CameraHandler is starting...");

  if (newFrameProducedCallback != nullptr)
  {
    mFrameProducedCallback_ = newFrameProducedCallback;
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

  equinox::trace("%s", "[CameraHandler] Camera handler thread start successful");
  return true;
}

void video_creek::CameraHandler::runCamera()
{
  equinox::trace("%s","[CameraHandler] CameraHandler thread is starting...");

  while(mContinueLoop_)
  {
    std::unique_lock<std::mutex> lock(mFramesGrabberThreadMutex_);

    equinox::trace("%s", "[CameraHandler] Camera handler thread is waiting for signal...");
    mConditionVariableFramesGrabberThread_.wait(lock, [this]()
    {
      return ((mNewFrameRequestedFlag_ == true) or (mContinueLoop_ == false));
    });

    if (mContinueLoop_ == false)
    {
      equinox::trace("%s", "[CameraHandler] CameraHandler thread is being stopped...");
      break;
    }

    if (mNewFrameRequestedFlag_ == true)
    {
      equinox::trace("%s", "[CameraHandler] New frame request signal received");
      mNewFrameRequestedFlag_ = false;
      if (mVideoCapture_.grab())
      {
        equinox::trace("%s", "[CameraHandler] New frame grabbed");

        std::lock_guard<std::mutex> guard(*mBufferLockMutex_);
        if (mVideoCapture_.retrieve(*mImageBuffer_))
        {
          equinox::trace("%s", "[CameraHandler] New frame retrieved");
          equinox::trace("[CameraHandler] Frame height: [%d]", mImageBuffer_->rows);
          equinox::trace("[CameraHandler] Frame width: [%d]", mImageBuffer_->cols);
          mFrameProducedCallback_();
          equinox::trace("%s", "[CameraHandler] Frame produced callback called");
        }
        else
        {
          equinox::trace("%s", "[CameraHandler] New frame retrieve failed");
        }
      }
      else
      {
        equinox::trace("%s", "[CameraHandler] New frame grab failed");
      }
    }

    equinox::trace("%s", "[CameraHandler] runCamera looping...");
  }
}

void video_creek::CameraHandler::requestNewFrame()
{
  equinox::trace("%s", "[CameraHandler] New frame request received");
  mNewFrameRequestedFlag_ = true;
  mConditionVariableFramesGrabberThread_.notify_all();
}
