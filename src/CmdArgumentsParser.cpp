/*
 * CmdArgumentsParser.cpp
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

#include <iostream>
#include <string>
#include <cstring>

#include <netinet/in.h>
#include <getopt.h>

#include "CmdArgumentsParser.h"
#include "EquinoxLogger.h"

#define MIN_NUMBER_OF_ARGUMENTS  2

static std::string logo_buffer = " __      ___     _             _____               _       "
    " \\ \\    / (_)   | |           / ____|             | |      "
    "  \\ \\  / / _  __| | ___  ___ | |     _ __ ___  ___| | _____"
    "   \\ \\/ / | |/ _` |/ _ \\/ _ \\| |    | '__/ _ \\/ _ \\ |/ / __|"
    "    \\  /  | | (_| |  __/ (_) | |____| | |  __/  __/   <\\__ \\ "
    "     \\/   |_|\\__,_|\\___|\\___/ \\_____|_|  \\___|\\___|_|\\_\\___/";

static std::string copyrights_buffer = "\t Version:    1.0\n"
    "\t Copyrights: Janusz Wolak\n"
    "\t e-mail:     januszvdm@gmail.com\n"
    "\t web:        github.com/jwolak\n\n";

void video_creek::CmdArgumentsParser::printHelp()
{
  std::string help_buffer = "\n\t#Menu:\n"
      "\t [--help]      or [-h] print help\n"
      "\t [--receiver]  or [-r] enable receiver mode\n"
      "\t [--sender]    or [-s] enable sender mode\n"
      "\t [--port]      or [-p] port number\n"
      "\t [--address]   or [-a] server IP address\n"
      "\t [--debug]     or [-d] debug logs level\n"
      "\n"
      "\t#Limitations:\n"
      "\t Port range        for [--port]  is: 1 - 65535\n"
      "\t Debug logs        for [--debug] is: [f] to log into a file, [c] to console or [b] to both\n"
      "\n"
      "\t#Examples of client and server set:\n"
      "\t Sender:     VideoCreeks -r -p 2024 -a 127.0.0.1\n"
      "\t Receiver:   VideoCreeks -s -p 2024\n"
      "\n"
      "\t Sender:     VideoCreeks --sender   --port 1024  --address 127.0.0.1\n"
      "\t Receiver:   VideoCreeks --receiver --port 1024\n"
      "\n"
      "\t#Remarks:\n"
      "\t No [--receiver] or [--sender] argument sets default role to receiver\n"
      "\t No [--port] argument for server and client sets default value to port 2024\n"
      "\t No [--address] argument for receiver sets default IP address to 0.0.0.0\n"
      "\n\t [IMPORTANT!] receiver mode has only port number argument allowed\n";

      std::cout << logo_buffer << std::endl;
      std::cout << help_buffer << std::endl;
      std::cout << copyrights_buffer << std::endl;
}


void video_creek::CmdArgumentsParser::parseArgs(int argc, char **argv)
{
  int flag = 0;
  int32_t portno = 0;
  char address_parameter[INET_ADDRSTRLEN] = {};

  if (argc < MIN_NUMBER_OF_ARGUMENTS) {

    std::cout << "[CmdArgumentsParser] No arguments" << std::endl;
    this->printHelp();
    exit(1);
  }

  static struct option longopts[] = {
      {"help",      no_argument,        NULL,  'h'},
      {"sender",    no_argument,        NULL,  's'},
      {"receiver",  no_argument,        NULL,  'r'},
      {"port",      required_argument,  NULL,  'p'},
      {"address",   required_argument,  NULL,  'a'},
      {"debug",     required_argument,  NULL,  'd'},
  };

  std::cout << std::endl;

  while ((flag = getopt_long(argc, argv, "hsrp:a:d:", longopts, NULL)) != -1) {
    switch (flag) {
      case 's':
        mCmdArguments_->setMode(Mode::SENDER);
        std::cout << "[CmdArgumentsParser] Set role to sender" << std::endl;
        break;

      case 'r':
        mCmdArguments_->setMode(Mode::RECEIVER);
        std::cout << "[CmdArgumentsParser] Set role to receiver" << std::endl;
        break;

      case 'p':
        portno = atoi(optarg);
        mCmdArguments_->setPort(portno);
        std::cout << "[CmdArgumentsParser] Port set to: " << portno << std::endl;
        break;

      case 'a':
        strncpy(address_parameter, optarg, INET_ADDRSTRLEN);
        mCmdArguments_->setDstAddress(std::string(address_parameter));
        break;

      case 'd':
        SET_LOG_LEVEL(equinox_logger::LogLevelType::LOG_LEVEL_DEBUG);
        LOG_DEBUG("%s", "[ParseArguments] Debug mode is enabled");
        LOG_WARNING("%s", "Debug mode enabled");

        if (strncmp(optarg, "f", 2) == 0) {
          SET_LOG_LOGGER_OUTPUT(equinox_logger::LogOutputType::FILE_LOG);
          LOG_DEBUG("%s", "[ParseArguments] Debug logs to file enabled");
          break;
        }

        if (strncmp(optarg, "c", 2) == 0) {
          SET_LOG_LOGGER_OUTPUT(equinox_logger::LogOutputType::CONSOLE);
          LOG_DEBUG("%s", "[ParseArguments] Debug logs to console enabled");
          break;
        }

        if (strncmp(optarg, "b", 2) == 0) {
          SET_LOG_LOGGER_OUTPUT(equinox_logger::LogOutputType::FILE_AND_CONSOLE);
          LOG_DEBUG("%s", "[ParseArguments] Debug logs to file and console enabled");
          break;

        } else {

          LOG_DEBUG("%s", "[ParseArguments] Invalid debug output direction parameter provided");
          LOG_ERROR("%s", "Invalid debug output direction parameter provided");
          return false;
        }
        break;

      case 'h':
      default:
        this->printHelp();
        exit(1);
    }
  }
}
