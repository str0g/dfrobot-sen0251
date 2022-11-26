//
// Created by lukasz on 26.11.2022.
//

#include "sen0251_exception.h"

const std::string splitter {"::"};

MyExceptionBase::MyExceptionBase(const char* in, const char* file, const char* function, int line):
  error(file + splitter + std::to_string(line) + splitter + function + splitter + in) {}

const char * MyExceptionBase::what () const throw () {
  return error.c_str();
}