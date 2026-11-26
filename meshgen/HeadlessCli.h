//
// HeadlessCli.h
//

#pragma once

#include <string>

bool IsHeadlessCommand(const std::string& command);
int RunHeadlessCli(int argc, char* argv[]);
