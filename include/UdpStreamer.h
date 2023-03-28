/*
 * UdpStreamer.h
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

#ifndef INCLUDE_UDPSTREAMER_H_
#define INCLUDE_UDPSTREAMER_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <functional>

#include <opencv2/core/mat.hpp>

#include "NetworkHandler.h"

namespace video_creek
{
class UdpStreamer
{
 public:
  UdpStreamer(std::shared_ptr<NetworkHandler> networkHandler)
  : mNetworkHandler_ { networkHandler }
  , mUdpStreamerThread_ { nullptr }
  , mConditionVariableUdpStreamerThread_ {}
  , mUdpStreamerMutex_ {}
  , mNewCompressFrameToBeSentFlag_ { false }
  , mCompressedFrameIsSentInfoCallback_ { nullptr }
  , mReceiverThread_ { nullptr }
  , mSenderThread_ { nullptr }
  , mNewFreameReceivedFlag_ { false }
  , mRequestSendFrameFlag_ { false }
  , mConditionVariableSenderThread_ {}
  , mSenderThreadMutex_ {}
  , mNewFreameSentFlag_ { false }
  , mContinueLoop_ { true }
  {
  }

  ~UdpStreamer();

  bool start(std::function<void(void)> compressedFrameIsSentInfoCallback);
  void stop();
  void send();

 private:
  std::shared_ptr<NetworkHandler> mNetworkHandler_;
  std::shared_ptr<std::thread> mUdpStreamerThread_;
  std::condition_variable mConditionVariableUdpStreamerThread_;
  std::mutex mUdpStreamerMutex_;
  std::atomic<bool> mNewCompressFrameToBeSentFlag_;
  std::function<void(void)> mCompressedFrameIsSentInfoCallback_;

  std::shared_ptr<std::thread> mReceiverThread_;
  std::shared_ptr<std::thread> mSenderThread_;
  std::atomic<bool> mNewFreameReceivedFlag_;
  std::atomic<bool> mRequestSendFrameFlag_;
  std::condition_variable mConditionVariableSenderThread_;
  std::mutex mSenderThreadMutex_;
  std::atomic<bool> mNewFreameSentFlag_;
  std::atomic<bool> mContinueLoop_;

  void runUdpStreamer();
  void runReceiver();
  void runSender();
};
} /*namespace video_creek*/

#endif /* INCLUDE_UDPSTREAMER_H_ */
