/*
 * CompressionHandler.cpp
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

#include "CompressionHandler.h"
#include "EquinoxLogger.h"

#include <opencv2/imgcodecs.hpp>

video_creek::CompressionHandler::~CompressionHandler()
{
  if(nullptr != mCompressionHandlerThread_)
  {
    mCompressionHandlerThread_->join();
  }
}

void video_creek::CompressionHandler::stop()
{
  equinox::trace("%s", "[CompressionHandler] CompressionHandler thread requested to be stopped");
  mContinueLoop_ = false;
  mConditionVariableCompressionHandlerThread_.notify_all();

  equinox::trace("%s", "[CompressionHandler] Waiting until CompressionHandler thread is stopped");
  if(nullptr != mCompressionHandlerThread_)
  {
    mCompressionHandlerThread_->join();
  }
  equinox::trace("%s", "[CompressionHandler] CompressionHandler thread is stopped");
}

bool video_creek::CompressionHandler::start(std::function<void(void)> compressedFrameIsReadyCallback)
{
  if (compressedFrameIsReadyCallback != nullptr)
  {
    mCompressedFrameIsReadyCallback_ = compressedFrameIsReadyCallback;
  }
  else
  {
    equinox::error("%s", "[CompressionHandler] Compressed frame is ready callback is null");
    return false;
  }

  mParametres_.push_back(cv::IMWRITE_JPEG_QUALITY);
  mParametres_.push_back(mCmdArguments_->getCompressionRatio());

  equinox::trace("[CompressionHandler] Image quality: IMWRITE_JPEG_QUALITY");
  equinox::trace("[CompressionHandler] Compressed ratio set to: [%d]", mCmdArguments_->getCompressionRatio());

  if (nullptr == (mCompressionHandlerThread_ = std::make_shared<std::thread>(&CompressionHandler::runCompressor, this)))
  {
    equinox::error("%s", "[CompressionHandler] Launch CompressionHandler thread failed");
    return false;
  }
  equinox::trace("%s", "[CompressionHandler] Launch CompressionHandler thread successful");

  return true;
}

void video_creek::CompressionHandler::runCompressor()
{
  while(true)
  {
    std::unique_lock<std::mutex> lock(mCompressionHandlerThreadMutex_);

    mConditionVariableCompressionHandlerThread_.wait(lock, [this]()
    {
      return ((mNewFrameToCompressFlag_ == true) or (mContinueLoop_ == false));
    });

    if (mContinueLoop_ == false)
    {
      equinox::trace("%s", "[CompressionHandler] CompressionHandler thread is being stopped...");
      break;
    }

    if (mNewFrameToCompressFlag_ == true)
    {
      mNewFrameToCompressFlag_ = false;
      equinox::trace("%s", "[CompressionHandler] New frame to compress signal received...");

      if (!imencode(".jpg", *mImageBuffer_, *mOutputBuffer_, mParametres_))
      {
        equinox::debug("%s", "[CompressionHandler] Compression of frame failed");
      }
      equinox::debug("%s", "[CompressionHandler] Compression of frame successful");

      mCompressedFrameIsReadyCallback_();
      equinox::debug("%s", "[CompressionHandler] Compressed frame is ready callback called");
    }

    equinox::trace("%s", "[CompressionHandler] runCompressor looping...");
  }
}

void video_creek::CompressionHandler::compressFrame()
{
  equinox::trace("%s", "[CompressionHandler] Compress the new frame requested");
  mNewFrameToCompressFlag_ = true;
  mConditionVariableCompressionHandlerThread_.notify_all();
}
