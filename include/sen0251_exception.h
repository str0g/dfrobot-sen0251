/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef DFROBOT_SEN0251_SEN0251_EXCEPTION_H
#define DFROBOT_SEN0251_SEN0251_EXCEPTION_H

#include <stdexcept>
#include <string>

class MyExceptionBase: public std::exception {
public:
  explicit MyExceptionBase(const char* in, const char* file, const char* function, int line);
  const char * what () const throw ();

private:
  std::string error;
};

#define THROW(type, msg) \
    throw type(msg, __FILE__, __FUNCTION__, __LINE__)

#define THROW_CLASS(name)\
class name : public MyExceptionBase { \
    public:\
        explicit name(const char* in, const char* file, const char* function, int line): \
            MyExceptionBase(in, file, function, line) {}\
};

THROW_CLASS(InvalidInputError);
THROW_CLASS(OperationError);
THROW_CLASS(OutOfRange);
THROW_CLASS(DeviceNotSupported);

#endif // DFROBOT_SEN0251_SEN0251_EXCEPTION_H
