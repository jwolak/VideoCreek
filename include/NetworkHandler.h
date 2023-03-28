/*
 * NetworkHandler.h
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

#ifndef INCLUDE_NETWORKHANDLER_H_
#define INCLUDE_NETWORKHANDLER_H_

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#include <memory>
#include <vector>

#include "CmdArguments.h"

namespace video_creek
{
class NetworkHandler
{
 public:
  NetworkHandler(std::shared_ptr<CmdArguments> cmdArguments, std::shared_ptr<std::vector<uint8_t>> outputBuffer)
  : mCmdArguments_ { cmdArguments }
  , mOutputBuffer_ { outputBuffer }
  , mSocket_ { -1 }
  , mDestAddr_ {}
  , mBindAddr_ {}
  , mRecentSenderAddr_ {}
  , mRecentSenderSize_ { 0 }
  {
  }

  ~NetworkHandler();

  bool start();
  void send();

 private:
  std::shared_ptr<CmdArguments> mCmdArguments_;
  std::shared_ptr<std::vector<uint8_t>> mOutputBuffer_;
  int32_t mSocket_;
  sockaddr_in mDestAddr_;
  sockaddr_in mBindAddr_;
  sockaddr mRecentSenderAddr_;
  socklen_t mRecentSenderSize_;
};
} /*namespace video_creek*/

#endif /* INCLUDE_NETWORKHANDLER_H_ */
