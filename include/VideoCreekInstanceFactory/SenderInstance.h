/*
 * SenderInstance.h
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

#ifndef INCLUDE_VIDEOCREEKINSTANCEFACTORY_SENDERINSTANCE_H_
#define INCLUDE_VIDEOCREEKINSTANCEFACTORY_SENDERINSTANCE_H_

#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <shared_mutex>

#include <opencv2/core/mat.hpp>

#include "IVideoCreekInstance.h"
#include "CameraHandler.h"
#include "CompressionHandler.h"
#include "UdpStreamer.h"
#include "CmdArguments.h"

namespace video_creek
{
class SenderInstance : public IVideoCreekInstance
{
 public:
  SenderInstance(std::shared_ptr<CmdArguments> cmdArguments)
  : mImageBufferLockMutex_ {}
  , mCmdArguments_ { cmdArguments }
  , mImageBuffer_ { std::make_shared<cv::Mat>() }
  , mEncodedVideoBuffer_ {}
  , mCameraHandler_ { std::make_shared<CameraHandler>(mImageBuffer_) }
  , mCompressionHandler_ { std::make_shared<CompressionHandler>(mImageBuffer_) }
  , mUdpStreamer_ { std::make_shared<UdpStreamer>(mImageBuffer_) }
  , mFrameSenderThread_ { nullptr }
  , mConditionVariableFramesSenderThread_ {}
  , mFramesSenderThreadMutex_ {}
  , mNewFrameReceivedFlag_ { false }
  , mCompressedFrameIsReadyFlag_ { false }
  , mInfoPacketIsSentFlag_ { false }
  {
  }

  ~SenderInstance();

  bool start() override;
  void newFrameProducedCallback();
  void compressedFrameIsReadyCallback();
  void compressedFrameIsSentInfoCallback();

 private:
  mutable std::shared_mutex mImageBufferLockMutex_;
  std::shared_ptr<CmdArguments> mCmdArguments_;
  std::shared_ptr<cv::Mat> mImageBuffer_;
  std::vector<uint8_t> mEncodedVideoBuffer_;
  std::shared_ptr<CameraHandler> mCameraHandler_;
  std::shared_ptr<CompressionHandler> mCompressionHandler_;
  std::shared_ptr<UdpStreamer> mUdpStreamer_;
  std::shared_ptr<std::thread> mFrameSenderThread_;
  std::condition_variable mConditionVariableFramesSenderThread_;
  std::mutex mFramesSenderThreadMutex_;
  std::atomic<bool> mNewFrameReceivedFlag_;
  std::atomic<bool> mCompressedFrameIsReadyFlag_;
  std::atomic<bool> mInfoPacketIsSentFlag_;

  void runSender();
};
} /*namespace video_creek*/

#endif /* INCLUDE_VIDEOCREEKINSTANCEFACTORY_SENDERINSTANCE_H_ */
