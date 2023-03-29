/*
 * CompressionHandler.h
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

#ifndef INCLUDE_COMPRESSIONHANDLER_H_
#define INCLUDE_COMPRESSIONHANDLER_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <functional>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "CmdArguments.h"

namespace video_creek
{
class CompressionHandler
{
 public:
  CompressionHandler(std::shared_ptr<cv::Mat> imageBuffer, std::shared_ptr<CmdArguments> cmdArguments_, std::shared_ptr<std::vector<uint8_t>> outputBuffer,
                     std::shared_ptr<std::mutex> bufferLockMutex)
  : mImageBuffer_ { imageBuffer } //TODO mutex for buffer?
  , mCmdArguments_ { cmdArguments_ }
  , mOutputBuffer_ { outputBuffer }
  , mBufferLockMutex_ { bufferLockMutex }
  , mCompressionHandlerThread_ { nullptr }
  , mConditionVariableCompressionHandlerThread_ {}
  , mCompressionHandlerThreadMutex_ {}
  , mNewFrameToCompressFlag_ { false }
  , mCompressedFrameIsReadyCallback_ { nullptr }
  , mContinueLoop_ { true }
  , mParametres_ { }
  {
  }

  ~CompressionHandler();

  bool start(std::function<void(void)> compressedFrameIsReadyCallback);
  void stop();
  void compressFrame();

 private:
  std::shared_ptr<cv::Mat> mImageBuffer_;
  std::shared_ptr<CmdArguments> mCmdArguments_;
  std::shared_ptr<std::vector<uint8_t>> mOutputBuffer_;
  std::shared_ptr<std::mutex> mBufferLockMutex_;
  std::shared_ptr<std::thread> mCompressionHandlerThread_;
  std::condition_variable mConditionVariableCompressionHandlerThread_;
  std::mutex mCompressionHandlerThreadMutex_;
  std::atomic<bool> mNewFrameToCompressFlag_;
  std::function<void(void)> mCompressedFrameIsReadyCallback_;
  std::atomic<bool> mContinueLoop_;
  std::vector<int32_t> mParametres_;

  void runCompressor();

};
} /*namespace video_creek*/

#endif /* INCLUDE_COMPRESSIONHANDLER_H_ */
