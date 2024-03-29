/*
 * VideoCreekCommon.h
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

#ifndef INCLUDE_VIDEOCREEKCOMMON_H_
#define INCLUDE_VIDEOCREEKCOMMON_H_

#include <cstdint>

#include "Mode.h"

namespace video_creek
{

const Mode kDefaultMode = Mode::RECEIVER;
const int32_t kDefaultPort = 1025;
const std::string kDefaultDstAddress = std::string("127.0.0.1");
const std::string kLogPrefix = std::string("VideoCreek");
const std::string kLogFileName = std::string("video_creek.log");
const int32_t kDefaultCompressionRatio = 80; /*[%]*/
const int32_t kDefaultCameraDeviceId = 0;
const double kDefaultHeight = 480;
const double kDefaultWidth = 640;
const std::string kDefaultBindAddress = std::string("0.0.0.0");

} /*namespace video_creek*/

#endif /* INCLUDE_VIDEOCREEKCOMMON_H_ */
