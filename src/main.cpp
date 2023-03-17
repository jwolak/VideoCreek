/*
 * main.cpp
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

#include <memory>
#include <iostream>
#include <cstdio>
#include <stdlib.h>

#include <csignal>
#include <csetjmp>

#include "CmdArguments.h"
#include "CmdArgumentsParser.h"
#include "VideoCreek.h"

static jmp_buf sigend_jmp_buf;

static void __attribute__ ((noreturn)) sigend_handler(int sig) {
  printf("\n%s %d %s", "Signal", sig, "caught\n");
  longjmp(sigend_jmp_buf, 1);
}

void catch_sigend(void (*handler)(int)) {
#ifdef SIGINT
  signal(SIGINT, handler);
#endif
#ifdef SIGTERM
  signal(SIGTERM, handler);
#endif
#ifdef SIGHUP
  signal(SIGHUP, handler);
#endif
}

int main(int argc, char **argv)
{
  std::shared_ptr<video_creek::CmdArguments> cmdArguments = std::make_shared<video_creek::CmdArguments>();
  video_creek::CmdArgumentsParser cmdArgumentsParser( cmdArguments );
  cmdArgumentsParser.parseArgs(argc, argv);

  video_creek::VideoCreek video_creek(cmdArguments);

  if (!video_creek.start())
  {
    std::cout << "[Main] Failed to start VideoCreek" << std::endl;
    exit(1);
  }

  catch_sigend(sigend_handler);
  if (setjmp(sigend_jmp_buf)) {
    video_creek.stop();
    exit(0);
  }

  return 0;
}


