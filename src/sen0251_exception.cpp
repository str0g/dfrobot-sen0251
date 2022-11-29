/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "sen0251_exception.h"

const std::string splitter {"::"};

MyExceptionBase::MyExceptionBase(const char* in, const char* file, const char* function, int line):
  error(file + splitter + std::to_string(line) + splitter + function + splitter + in) {}

const char * MyExceptionBase::what () const throw () {
  return error.c_str();
}