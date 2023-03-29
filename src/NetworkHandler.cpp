/*
 * NetworkHandler.cpp
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

#include <unistd.h>
#include <cstring>

#include <arpa/inet.h>

#include "NetworkHandler.h"
#include "EquinoxLogger.h"
#include "VideoCreekCommon.h"

video_creek::NetworkHandler::~NetworkHandler()
{
  if (mSocket_ > 0)
  {
    close(mSocket_);
    equinox::debug("%s", "[NetworkHandler] Socket close successful");
  }
}

bool video_creek::NetworkHandler::start()
{

  if (0 > (mSocket_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0)))
  {
    equinox::error("%s", "[NetworkHandler] Open socket failed");
    return false;
  }
  equinox::trace("%s", "[NetworkHandler] Open socket successful");

  memset((char*)&mBindAddr_, 0, sizeof(mBindAddr_));
  if ( 1 != (inet_pton(AF_INET, static_cast<const char*>(kDefaultBindAddress.c_str()), &(mBindAddr_.sin_addr))))
  {
    equinox::error("%s", "[NetworkHandler] Bind address conversion of IP address from text to binary form failed");
    return false;
  }
  equinox::trace("%s", "[NetworkHandler] Conversion of IPv4 and IPv6 addresses from text to binary form successful");

  mBindAddr_.sin_family = AF_INET;
  mBindAddr_.sin_port = htons(mCmdArguments_->getPort());

  if (bind(mSocket_, (sockaddr*)&mBindAddr_, sizeof(mBindAddr_)) < 0)
  {
    equinox::error("%s", "[NetworkHandler] Bind failed");
    close(mSocket_);
    equinox::debug("%s", "[NetworkHandler] Socket closed");
    return false;
  }
  equinox::trace("%s", "[NetworkHandler] Bind successful");

  memset((char*)&mDestAddr_, 0, sizeof(mDestAddr_));
  memset((char*)&mRecentSenderAddr_, 0, sizeof(mRecentSenderAddr_));
  mRecentSenderSize_ = sizeof(mRecentSenderAddr_);

  memset((char*)&mDestAddr_, 0, sizeof(mDestAddr_));
  mDestAddr_.sin_family = AF_INET;
  mDestAddr_.sin_port = htons(mCmdArguments_->getPort());
  if (1 != (inet_pton(AF_INET, mCmdArguments_->getDstAddress().c_str(), &mDestAddr_.sin_addr)))
  {
    equinox::error("%s", "[NetworkHandler] Destination address conversion of IPv4 and IPv6 addresses from text to binary form failed");
    return false;
  }
  equinox::error("%s", "[NetworkHandler] Destination address conversion of IP address from text to binary form successful");

  equinox::trace("%s", "[NetworkHandler] Start successful");
  return true;
}

void video_creek::NetworkHandler::send()
{
  std::lock_guard<std::mutex> guard(*mBufferLockMutex_);

  if ( 0 < (sendto(mSocket_, static_cast<uint8_t*>(&mOutputBuffer_->operator [](0)), mOutputBuffer_->size(), 0, (sockaddr*)&mDestAddr_, sizeof(mDestAddr_))))
  {
    equinox::error("%s", "[NetworkHandler] Send frame failed");
    return;
  }

  equinox::error("%s", "[NetworkHandler] Send frame successful");
}
