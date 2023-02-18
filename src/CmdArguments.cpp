/*
 * CmdArguments.cpp
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

#include "CmdArguments.h"
#include "EquinoxLogger.h"

void video_creek::CmdArguments::setMode(video_creek::Mode modeToSet)
{
  equinox::trace("[CmdArguments] Mode is set to: [%s]", modeToSet == video_creek::Mode::RECEIVER ? "Receiver" : "Sender");
  mMode_ = modeToSet;
}

video_creek::Mode video_creek::CmdArguments::getMode()
{
  return mMode_;
}

void video_creek::CmdArguments::setPort(int32_t portToSet)
{
  equinox::trace("[CmdArguments] Port set to: [%d]", portToSet);
  mPort_ = portToSet;
}

int32_t video_creek::CmdArguments::getPort()
{
  return mPort_;
}

void video_creek::CmdArguments::setDstAddress(const std::string &addressToSet)
{
  equinox::trace("[CmdArguments] Destination address set to: [%s]", addressToSet);
  mDstAddress_ = addressToSet;
}

std::string video_creek::CmdArguments::getDstAddress()
{
  return mDstAddress_;
}

void video_creek::CmdArguments::setCompressionRatio(int32_t compresRatioToSet)
{
  equinox::trace("[CmdArguments] Compression ratio set to: [%d]", compresRatioToSet);
  nCompressionRatio_ = compresRatioToSet;
}

int32_t video_creek::CmdArguments::getCompressionRatio()
{
  return nCompressionRatio_;
}
