// Copyright 2014 Withrobot, Inc
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Robotnik Automation S.L.L. (2023) nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Forked from: https://github.com/withrobot/myAHRS_plus/blob/17b3ccd6af62eca3930f393171e5df1e3d133531/common_cpp/myahrs_plus.hpp
 * Unkown distribution license, defaulting to BSD 2.0
 */

#ifndef MYAHRS_ROS2_DRIVER__MYAHRS_PLUS_HPP_
#define MYAHRS_ROS2_DRIVER__MYAHRS_PLUS_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <exception>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <functional>

#ifdef WIN32
#define _AFXDLL
#include <afx.h>
#include <windows.h>
#include <cctype>

// sprintf
#pragma warning(disable : 4996)
// warning C4355: 'this' : used in base member initializer list
#pragma warning(disable : 4355)
// warning C4244: '=' : conversion from 'double' to 'float', possible loss of data
#pragma warning(disable : 4244)

#define DBG_PRINTF(x, ...) {if (x) {printf(__VA_ARGS__);}}

#else  // LINUX
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>

#pragma GCC diagnostic ignored "-Wformat"
#define DBG_PRINTF(x, ...)  {if (x) {printf(__VA_ARGS__);}}
#endif  // WIN32

/*
 *  Version
 */
#define WITHROBOT_MYAHRS_PLUS_SDK_VERSION "myahrs+ sdk. ver. 1.01"

/*
 *  debugging sw
 */
#define DEBUG_PLATFORM false
#define DEBUG_ASCII_PROTOCOL false
#define DEBUG_BINARY_PROTOCOL false
#define DEBUG_MYAHRS_PLUS false

namespace WithRobot
{

class myAhrsException
{
public:
  std::string err;
  explicit myAhrsException(std::string e = "")
  : err(e) {}
  virtual ~myAhrsException() {}

  const char * what() const throw()
  {
    return err.c_str();
  }
};

/**********************************************************************************************************
 *
 * Platform abstraction
 *
 **********************************************************************************************************/

#ifdef WIN32
class Platform
{
public:
  class SerialPort
  {
    std::string port_name;
    unsigned int baudrate;

    HANDLE m_hIDComDev;
    BOOL m_bOpened;

public:
    explicit SerialPort(const char * port = "COM3", unsigned int brate = 115200)
    : port_name(port), baudrate(brate),
      m_hIDComDev(NULL), m_bOpened(FALSE)
    {
    }

    ~SerialPort()
    {
      Close();
    }

    bool Open(const char * port, int brate)
    {
      port_name = port;
      baudrate = brate;
      return Open();
    }

    bool Open()
    {
      char szPort[32];
      snprintf(szPort, sizeof(szPort), "\\\\.\\%s", port_name.c_str());
      CString port_str = CString::CStringT(CA2CT(szPort));

      DBG_PRINTF(DEBUG_PLATFORM, "portname : %s, baudrate %d\n", szPort, baudrate);

      m_hIDComDev = CreateFile(
        (LPCTSTR)port_str,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

      if (m_hIDComDev == NULL) {
        DBG_PRINTF(DEBUG_PLATFORM, "ERROR : m_hIDComDev == NULL\n");
        return false;
      }

      DCB dcb = {0};

      if (!GetCommState(m_hIDComDev, &dcb)) {
        // error getting state
        CloseHandle(m_hIDComDev);
        printf("ERROR : GetCommState()\n");
        return false;
      }

      dcb.DCBlength = sizeof(DCB);
      GetCommState(m_hIDComDev, &dcb);
      dcb.BaudRate = baudrate;
      dcb.ByteSize = 8;

      if (!SetCommState(m_hIDComDev, &dcb)) {
        // error setting serial port state
        CloseHandle(m_hIDComDev);
        DBG_PRINTF(DEBUG_PLATFORM, "ERROR : SetCommState()\n");
        return false;
      }

      COMMTIMEOUTS CommTimeOuts;

      CommTimeOuts.ReadIntervalTimeout = 1;
      CommTimeOuts.ReadTotalTimeoutMultiplier = 1;
      CommTimeOuts.ReadTotalTimeoutConstant = 1;
      CommTimeOuts.WriteTotalTimeoutMultiplier = 1;
      CommTimeOuts.WriteTotalTimeoutConstant = 10;

      if (!SetCommTimeouts(m_hIDComDev, &CommTimeOuts)) {
        CloseHandle(m_hIDComDev);
        DBG_PRINTF(DEBUG_PLATFORM, "ERROR : SetCommTimeouts()\n");
        return false;
      }

      m_bOpened = TRUE;

      return m_bOpened == TRUE;
    }

    void Close()
    {
      if (!m_bOpened || m_hIDComDev == NULL) {
        return;
      }

      CloseHandle(m_hIDComDev);
      m_bOpened = FALSE;
      m_hIDComDev = NULL;
    }

    int Read(unsigned char * buf, unsigned int buf_len)
    {
      if (!m_bOpened || m_hIDComDev == NULL) {
        return -1;
      }

      BOOL bReadStatus;
      DWORD dwBytesRead, dwErrorFlags;
      COMSTAT ComStat;

      ClearCommError(m_hIDComDev, &dwErrorFlags, &ComStat);

      if (ComStat.cbInQue <= 0) {
        dwBytesRead = 1;
      } else {
        dwBytesRead = static_cast<DWORD>(ComStat.cbInQue);
      }

      if (buf_len < static_cast<int>(dwBytesRead)) {
        dwBytesRead = static_cast<DWORD>(buf_len);
      }

      bReadStatus = ReadFile(m_hIDComDev, buf, dwBytesRead, &dwBytesRead, NULL);
      if (!bReadStatus) {
        // if( GetLastError() == ERROR_IO_PENDING ){
        //     WaitForSingleObject( m_OverlappedRead.hEvent, 2000 );
        //     return( static_cast<int>( dwBytesRead ));
        // }
        return 0;
      }

      return static_cast<int>(dwBytesRead);
    }

    int Write(unsigned char * data, unsigned int data_len)
    {
      BOOL bWriteStat;
      DWORD dwBytesWritten = 0;

      if (m_bOpened != TRUE) {
        return -1;
      }

      bWriteStat = WriteFile(m_hIDComDev, (LPSTR)data, data_len, &dwBytesWritten, NULL);
      if (!bWriteStat) {
        if (GetLastError() != ERROR_IO_PENDING) {
          // WriteFile failed, but it isn't delayed. Report error and abort.
          return dwBytesWritten;
        } else {
          // Write is pending... 여기로 빠질리는 없지만서도...
          // Sleep(10);
          // retry;
          return dwBytesWritten;
        }
      } else {
        return dwBytesWritten;
      }
    }

    int Flush()
    {
      int len = 0, count = 0;
      unsigned char buf[256];
      while ((len = Read(buf, sizeof(buf))) > 0) {
        count += len;
      }
      return count;
    }
  };

  class Mutex
  {
    CRITICAL_SECTION cs;

public:
    Mutex()
    {
      InitializeCriticalSection(&cs);
    }

    ~Mutex()
    {
      DeleteCriticalSection(&cs);
    }

    inline void lock()
    {
      EnterCriticalSection(&cs);
    }

    inline void unlock()
    {
      LeaveCriticalSection(&cs);
    }
  };

  class Event
  {
    HANDLE event;

public:
    Event()
    {
      event = CreateEvent(
        NULL,
        TRUE,                    // bManualReset
        FALSE,                   // bInitialState
        _T("Event"));
    }

    ~Event()
    {
      CloseHandle(event);
    }

    inline bool wait(int timeout_msec = -1)
    {
      if (timeout_msec < 0) {
        timeout_msec = INFINITE;
      }
      DWORD res = WaitForSingleObject(event, timeout_msec);
      ResetEvent(event);
      return res == WAIT_OBJECT_0;
    }

    inline bool set()
    {
      return SetEvent(event) == TRUE;
    }
  };

  class Thread
  {
    DWORD thread_id;
    HANDLE thread;

public:
    Thread()
    : thread(NULL), thread_id(0) {}

    bool start(void *(*thread_proc)(void *), void * arg, size_t stack_size = 16 * 1024)
    {
      thread = CreateThread(
        NULL, static_cast<DWORD>(stack_size), (LPTHREAD_START_ROUTINE)thread_proc, arg, 0,
        &thread_id);
      return thread != NULL;
    }

    void join()
    {
      if (thread != NULL) {
        WaitForSingleObject(thread, INFINITE);
        CloseHandle(thread);
      }
      thread = NULL;
      thread_id = 0;
    }
  };

  static void msleep(unsigned int msec)
  {
    Sleep(msec);
  }
};

#else
/*
 * Unix-Like OS (Linux/OSX)
 */
class Platform
{
public:
  class SerialPort
  {
    std::string port_name;   // ex) "/dev/tty.usbmodem14241"
    int port_fd;
    unsigned int baudrate;

public:
    explicit SerialPort(const char * port = "", unsigned int brate = 115200)
    : port_name(port), baudrate(brate), port_fd(-1)
    {
    }

    ~SerialPort()
    {
      Close();
    }

    bool Open(const char * port, int brate)
    {
      port_name = port;
      baudrate = brate;
      return Open();
    }

    bool Open()
    {
      int fd = 0;
      struct termios options;

      if (port_fd > 0) {
        return true;
      }

      fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd < 0) {
        return false;
      }

      fcntl(fd, F_SETFL, 0);     // clear all flags on descriptor, enable direct I/O
      tcgetattr(fd, &options);   // read serial port options

      cfsetspeed(&options, baudrate);
      cfmakeraw(&options);

      options.c_cflag |= CREAD | CLOCAL;   // turn on READ
      options.c_cflag |= CS8;
      options.c_cc[VMIN] = 0;
      options.c_cc[VTIME] = 1;

      // set the new port options
      tcsetattr(fd, TCSANOW, &options);

      // flush I/O buffers
      tcflush(fd, TCIOFLUSH);

      port_fd = fd;

      return fd > 0;
    }

    void Close()
    {
      if (port_fd > 0) {
        close(port_fd);
        port_fd = -1;
      }
    }

    int Read(unsigned char * buf, unsigned int buf_len)
    {
      if (port_fd < 0) {
        return -1;
      }

      int n = read(port_fd, buf, buf_len - 1);
      if (n > 0) {
        buf[n] = 0;
      }

      return n;
    }

    int Write(unsigned char * data, unsigned int data_len)
    {
      if (port_fd < 0) {
        return -1;
      }

      return write(port_fd, data, data_len);
    }

    int Flush()
    {
      if (port_fd < 0) {
        return -1;
      }

      // flush I/O buffers
      return tcflush(port_fd, TCIOFLUSH);
    }
  };

  class Mutex
  {
    pthread_mutex_t mutex;

public:
    Mutex()
    {
      pthread_mutex_init(&mutex, NULL);
    }

    ~Mutex()
    {
      pthread_mutex_destroy(&mutex);
    }

    inline void lock()
    {
      pthread_mutex_lock(&mutex);
    }

    inline void unlock()
    {
      pthread_mutex_unlock(&mutex);
    }
  };

  class Event
  {
    pthread_mutex_t mutex;
    pthread_cond_t cond;

public:
    Event()
    {
      pthread_mutex_init(&mutex, NULL);
      pthread_cond_init(&cond, NULL);
    }

    ~Event()
    {
      pthread_mutex_destroy(&mutex);
      pthread_cond_destroy(&cond);
    }

    inline bool wait(int timeout_msec = -1)
    {
      bool res = true;
      pthread_mutex_lock(&mutex);
      if (timeout_msec > 0) {
        struct timeval tv;
        struct timespec ts;

        gettimeofday(&tv, NULL);

        ts.tv_sec = time(NULL) + timeout_msec / 1000;
        ts.tv_nsec = tv.tv_usec * 1000 + 1000 * 1000 * (timeout_msec % 1000);
        ts.tv_sec += ts.tv_nsec / (1000 * 1000 * 1000);
        ts.tv_nsec %= (1000 * 1000 * 1000);

        int n = pthread_cond_timedwait(&cond, &mutex, &ts);
        res = (n == 0);   // n == 0 : signaled, n == ETIMEDOUT
      } else {
        pthread_cond_wait(&cond, &mutex);
        res = true;
      }
      pthread_mutex_unlock(&mutex);
      return res;
    }

    inline bool set()
    {
      pthread_mutex_lock(&mutex);
      pthread_cond_signal(&cond);
      pthread_mutex_unlock(&mutex);
      return true;
    }
  };

  class Thread
  {
    pthread_t thread;

public:
    Thread()
    : thread(0) {}

    bool start(void *(*thread_proc)(void *), void * arg, size_t stack_size = 16 * 1024)
    {
      pthread_attr_t attr;
      size_t stacksize;
      pthread_attr_init(&attr);

      // pthread_attr_getstacksize(&attr, &stacksize);

      pthread_attr_setstacksize(&attr, stack_size);
      int res = pthread_create(&thread, &attr, thread_proc, reinterpret_cast<void *>(arg));
      if (res != 0) {
        thread = 0;
      }

      return res == 0;
    }

    void join()
    {
      if (thread != 0) {
        pthread_join(thread, NULL);
      }
      thread = 0;
    }
  };

  static void msleep(unsigned int msec)
  {
    usleep(msec * 1000);
  }
};   // Platform

#endif  // #ifdef WIN32

/**********************************************************************************************************
 *
 * platform independent implementation
 *
 **********************************************************************************************************/

class LockGuard
{
  Platform::Mutex & mutex;

public:
  explicit LockGuard(Platform::Mutex & m)
  : mutex(m)
  {
    mutex.lock();
  }

  ~LockGuard()
  {
    mutex.unlock();
  }
};

class StringUtil
{
public:
  static void replace(std::string & src, std::string s1, std::string s2)
  {
    size_t ofs = 0;

    do {
      ofs = src.find(s1, ofs);
      if (ofs == -1) {
        return;
      }
      src.replace(ofs, s1.length(), s2);
      ofs += s2.length();
    } while (1);
  }

#if 0
  // trim from start
  static inline std::string & ltrim(std::string & s)
  {
    s.erase(
      s.begin(),
      std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
  }

  // trim from end
  static inline std::string & rtrim(std::string & s)
  {
    s.erase(
      std::find_if(
        s.rbegin(), s.rend(), std::not1(
          std::ptr_fun<int, int>(
            std::isspace))).base(), s.end());
    return s;
  }

  // trim from both ends
  static inline std::string & strip(std::string & s)
  {
    return ltrim(rtrim(s));
  }
#endif

  static int split(
    std::vector<std::string> & token_list, const char * c_str, char delimiter,
    int token_max = -1)
  {
    std::stringstream is(c_str);
    std::string token;

    bool check_max = (token_max > 0);

    token_list.clear();
    while (getline(is, token, delimiter)) {
      token_list.push_back(token);
      if (check_max) {
        if (--token_max <= 0) {
          break;
        }
      }
    }

    return token_list.size();
  }

  static std::string join(std::vector<std::string> & str_list, const std::string & delimiter)
  {
    std::ostringstream s;
    for (size_t i = 0; i < str_list.size(); i++) {
      if (i != 0) {
        s << delimiter;
      }
      s << str_list[i];
    }
    return s.str();
  }

  template<typename T>
  static void to_string_list(std::vector<std::string> & str_list, T * array, size_t array_num)
  {
    for (size_t i = 0; i < array_num; i++) {
      std::ostringstream s;
      s << array[i];
      str_list.push_back(s.str());
    }
  }

  static int extract_attributes(
    std::map<std::string, std::string> & attribute_list,
    std::vector<std::string> & tokens)
  {
    std::vector<std::string> attribute_pair;

    for (size_t i = 0; i < tokens.size(); i++) {
      // "attribute_name=value"
      if (split(attribute_pair, tokens[i].c_str(), '=') == 2) {
        attribute_list[attribute_pair[0]] = attribute_pair[1];
      }
    }

    return attribute_list.size();
  }
};

struct FrameBuffer
{
  unsigned char buffer[1024];
  size_t offset;

  FrameBuffer()
  {
    reset();
  }

  void push(unsigned char byte)
  {
    if (offset < sizeof(buffer)) {
      buffer[offset++] = byte;
    }
  }

  void reset()
  {
    memset(buffer, 0, sizeof(buffer));
    offset = 0;
  }
};

/**************************************************************************
 *
 * COMMUNICATION PROTOCOL (ASCII)
 *
 **************************************************************************/

class iAsciiProtocol
{
public:
  static const uint8_t MSG_HDR_SENSOR_DATA = '$';
  static const uint8_t MSG_HDR_COMMAND = '@';
  static const uint8_t MSG_HDR_RESPONSE = '~';
  static const uint8_t MSG_CRC_DELIMITER = '*';
  static const uint8_t MSG_TAIL_CR = '\r';
  static const uint8_t MSG_TAIL_LF = '\n';

private:
  FrameBuffer frame_buffer;
  bool debug;

public:
  iAsciiProtocol()
  : debug(DEBUG_ASCII_PROTOCOL)
  {
  }
  virtual ~iAsciiProtocol() {}

protected:
  void push_byte(unsigned char c)
  {
    switch (c) {
      // start of frame
      case MSG_HDR_SENSOR_DATA:
      case MSG_HDR_RESPONSE:
      case MSG_HDR_COMMAND:
        frame_buffer.reset();
        frame_buffer.push(c);
        break;

      // end of frame
      case MSG_TAIL_LF:
        if (frame_buffer.offset > 0) {
          parse_message(reinterpret_cast<const char *>(frame_buffer.buffer));
        }
        frame_buffer.reset();
        break;

      case MSG_TAIL_CR:
        // nothing to do
        break;

      default:
        frame_buffer.push(c);
    }
  }

  virtual void update_attributes(std::vector<std::string> & tokens) = 0;

private:
  bool parse_message(const char * ascii_frame)
  {
    DBG_PRINTF(debug, "## ASC FRAME : [%s]\n", ascii_frame);

    std::vector<std::string> tokens;
    if (StringUtil::split(tokens, ascii_frame, '*') != 2) {
      return false;
    }

    std::string & str_message = tokens[0];
    int crc_from_sensor = std::stoul(tokens[1].c_str(), nullptr, 16);

    uint8_t crc_calc = 0;
    for (size_t i = 0; i < str_message.length(); i++) {
      crc_calc ^= str_message[i];
    }

    DBG_PRINTF(
      debug, "\t= MSG %s, CRC %X, CRC_C %X\n", tokens[0].c_str(), crc_from_sensor,
      crc_calc);

    if (crc_calc == crc_from_sensor) {
      StringUtil::split(tokens, str_message.c_str(), ',');
      update_attributes(tokens);
      return true;
    } else {
      return false;
    }
  }
};

/**************************************************************************
 *
 * COMMUNICATION PROTOCOL (BINARY)
 *
 **************************************************************************/

class FilterByteStuffing
{
public:
  enum
  {
    STX = 0x02,
    ETX = 0x03,
    DLE = 0x10,
  };

  enum ReturnCode
  {
    STATE_NOP = 0,
    STATE_BUSY,
    STATE_COMPLETE,   // RECEIVE COMPLETE
    STATE_ERROR,
  };

private:
  bool state_receiving;
  ReturnCode last_state;

  FrameBuffer & stream;
  uint8_t crc_calc;
  uint16_t accumulater;

  bool debug;

  ReturnCode (FilterByteStuffing::* receiver)(uint8_t);

public:
  explicit FilterByteStuffing(FrameBuffer & s)
  : stream(s), crc_calc(0), accumulater(0), state_receiving(false), last_state(STATE_NOP), debug(
      DEBUG_BINARY_PROTOCOL)
  {
    receiver = &FilterByteStuffing::state_data;
  }

  ~FilterByteStuffing() {}

  ReturnCode operator()(uint8_t byte)
  {
    DBG_PRINTF(debug, "new byte %02X\n", byte);
    last_state = (this->*receiver)(byte);
    return last_state;
  }

  bool is_busy()
  {
    return last_state == STATE_BUSY;
  }

private:
  void clear_all_states()
  {
    state_receiving = false;
    stream.reset();
    crc_calc = 0;
    accumulater = 0;
  }

  bool check_crc()
  {
    uint8_t crc_rcv = accumulater & 0x00FF;
    DBG_PRINTF(debug, "### CRC_R %02X, CRC_C %02X\n", crc_rcv, crc_calc);
    return crc_rcv == crc_calc;
  }

  void push_data(uint8_t byte)
  {
    stream.push(byte);

    accumulater <<= 8;
    accumulater |= byte;
    uint8_t prv_byte = ((accumulater & 0xFF00) >> 8);
    crc_calc ^= prv_byte;
  }

  ReturnCode state_data(uint8_t byte)
  {
    if (byte == DLE) {
      receiver = &FilterByteStuffing::state_control;
      return STATE_BUSY;
    } else {
      if (state_receiving) {
        push_data(byte);
        return STATE_BUSY;
      } else {
        return STATE_NOP;
      }
    }
  }

  ReturnCode state_control(uint8_t byte)
  {
    ReturnCode res;

    switch (byte) {
      case STX:
        clear_all_states();
        state_receiving = true;
        res = STATE_BUSY;
        break;

      case ETX:
        state_receiving = false;
        res = check_crc() ? STATE_COMPLETE : STATE_ERROR;
        break;

      case DLE:
        push_data(byte);
        res = STATE_BUSY;
        break;

      default:
        clear_all_states();
        state_receiving = true;
        res = STATE_ERROR;
    }

    receiver = &FilterByteStuffing::state_data;
    return res;
  }
};   // FilterDleToPlain

class iNodeParser
{
public:
  enum Tag
  {
    // value type
    TAG_TYPE_NONE = 0,
    TAG_TYPE_INT8 = 1,
    TAG_TYPE_UINT8 = 2,
    TAG_TYPE_INT16 = 3,
    TAG_TYPE_UINT16 = 4,
    TAG_TYPE_INT32 = 5,
    TAG_TYPE_UINT32 = 6,
    TAG_TYPE_INT64 = 7,
    TAG_TYPE_UINT64 = 8,
    TAG_TYPE_FLOAT32 = 9,
    TAG_TYPE_FLOAT64 = 10,
    TAG_TYPE_STRING = 11,

    // node attribute
    TAG_HAS_LEAF_NODES = (0x01 << 7),
    TAG_NEXT_NODE_EXIST = (0x01 << 6),
    TAG_LIST_NODE = (0x01 << 5),

    TAG_TYPE_MASK = 0x0F,
  };

protected:
  struct Varient
  {
    Varient()
    : type(TAG_TYPE_NONE) {}
    uint8_t type;
    union {
      int8_t i8;
      uint8_t ui8;
      int16_t i16;
      uint16_t ui16;
      int32_t i32;
      uint32_t ui32;
      int64_t i64;
      uint64_t ui64;
      float f32;
      double f64;
    } value;

    template<typename T>
    bool set(uint8_t t, T v)
    {
      type = t;
      switch (type) {
        case TAG_TYPE_INT8:
          value.i8 = (int8_t)v;
          break;
        case TAG_TYPE_UINT8:
          value.ui8 = (uint8_t)v;
          break;
        case TAG_TYPE_INT16:
          value.i16 = (int16_t)v;
          break;
        case TAG_TYPE_UINT16:
          value.ui16 = (uint16_t)v;
          break;
        case TAG_TYPE_INT32:
          value.i32 = (int32_t)v;
          break;
        case TAG_TYPE_UINT32:
          value.ui32 = (uint32_t)v;
          break;
        case TAG_TYPE_INT64:
          value.i64 = (int64_t)v;
          break;
        case TAG_TYPE_UINT64:
          value.ui64 = (uint64_t)v;
          break;
        case TAG_TYPE_FLOAT32:
          value.f32 = static_cast<float>(v);
          break;
        case TAG_TYPE_FLOAT64:
          value.f64 = static_cast<double>(v);
          break;
        default:
          return false;
      }
      return true;
    }
  };

public:
  struct Node
  {
    std::string name;
    std::vector<Varient> list;
  };

private:
  struct Stream
  {
    unsigned char * buffer;
    size_t pos;
    size_t length;
    bool debug;

    Stream(unsigned char * s, size_t l)
    : buffer(s), length(l), pos(0), debug(DEBUG_BINARY_PROTOCOL)
    {
      print_buffer();
    }

    int getc()
    {
      if (pos < length) {
        return buffer[pos++];
      } else {
        return -1;
      }
    }

    int peek()
    {
      if (pos < length) {
        return buffer[pos];
      } else {
        return -1;
      }
    }

    template<typename T>
    void read_value(T & value)
    {
      T * v = reinterpret_cast<T *>(&buffer[pos]);
      pos += sizeof(T);
      value = *v;
    }

    bool read_value(uint8_t value_type, Varient & v)
    {
      switch (value_type) {
        case TAG_TYPE_INT8:
          {
            int8_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_UINT8:
          {
            uint8_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_INT16:
          {
            int16_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_UINT16:
          {
            uint16_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_INT32:
          {
            int32_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_UINT32:
          {
            uint32_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_INT64:
          {
            int64_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_UINT64:
          {
            uint64_t value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_FLOAT32:
          {
            float value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        case TAG_TYPE_FLOAT64:
          {
            double value;
            read_value(value);
            v.set(value_type, value);
          }
          break;
        default:
          return false;
      }
      return true;
    }

    void read_list(uint8_t value_type, std::vector<Varient> & list, size_t count)
    {
      Varient v;
      for (size_t i = 0; i < count; i++) {
        if (read_value(value_type, v) == true) {
          list.push_back(v);
        }
      }
    }

    std::string read_string()
    {
      std::string str((const char *)&buffer[pos]);
      pos += str.length() + 1;   // '\0'
      return str;
    }

private:
    void print_buffer()
    {
      if (debug) {
        for (size_t i = 0; i < length; i++) {
          DBG_PRINTF(debug, "%02X ", buffer[i]);
        }
        DBG_PRINTF(debug, "\n");
      }
    }
  };

  Stream istream;
  bool debug;

  std::vector<Node> node_list;

public:
  explicit iNodeParser(unsigned char * stream = 0, size_t stream_len = 0)
  : istream(stream, stream_len), debug(false) {}
  virtual ~iNodeParser() {}

  void parse()
  {
    read_nodes();
    new_node(node_list);
  }

  virtual void new_node(std::vector<Node> & node_list) = 0;

private:
  void read_nodes()
  {
    if (istream.length == 0 || istream.peek() < 0) {
      return;
    }

    uint8_t tag = istream.getc();
    uint8_t type_of_value = (tag & TAG_TYPE_MASK);

    Node node;
    node.name = istream.read_string();
    if (type_of_value) {
      if (tag & TAG_LIST_NODE) {
        uint16_t count = 0;
        istream.read_value(count);
        istream.read_list(type_of_value, node.list, count);
      } else {
        Varient value;
        istream.read_value(type_of_value, value);
        node.list.push_back(value);
      }
    }

    node_list.push_back(node);

    if ((tag & TAG_HAS_LEAF_NODES) || (tag & TAG_NEXT_NODE_EXIST)) {
      read_nodes();
    }
  }
};

class iBinaryProtocol
{
  FilterByteStuffing filter_byte_stuffing;
  FrameBuffer binary_stream;

  class BinaryNodeParser : public iNodeParser
  {
    iBinaryProtocol * protocol;

public:
    BinaryNodeParser(iBinaryProtocol * s, unsigned char * stream, size_t stream_len)
    : protocol(s), iNodeParser(stream, stream_len) {}
    void new_node(std::vector<Node> & node_list)
    {
      protocol->update_attributes(node_list);
    }
  };

public:
  iBinaryProtocol()
  : filter_byte_stuffing(binary_stream) {}
  virtual ~iBinaryProtocol() {}

protected:
  void push_byte(unsigned char c)
  {
    if (filter_byte_stuffing(c) == FilterByteStuffing::STATE_COMPLETE) {
      BinaryNodeParser parser(this, binary_stream.buffer, binary_stream.offset);
      parser.parse();
    }
  }

  bool is_receiving()
  {
    return filter_byte_stuffing.is_busy();
  }

  virtual void update_attributes(std::vector<iNodeParser::Node> & node_list) = 0;
};   // BinaryProtocol

/**************************************************************************
 *
 * COMMUNICATION PROTOCOL
 *
 **************************************************************************/

class iProtocol : public iAsciiProtocol, public iBinaryProtocol
{
public:
  iProtocol() {}
  virtual ~iProtocol() {}

  bool feed(unsigned char * data, int data_len)
  {
    if (!data || data_len <= 0) {
      return false;
    }

    for (int i = 0; i < data_len; i++) {
      iBinaryProtocol::push_byte(data[i]);
      if (iBinaryProtocol::is_receiving() == false) {
        iAsciiProtocol::push_byte(data[i]);
      }
    }

    return true;
  }
};

/**************************************************************************
 *
 * ATTITUDE REPRESENTATIONS
 *
 **************************************************************************/

struct EulerAngle
{
  double roll, pitch, yaw;

  explicit EulerAngle(double r = 0, double p = 0, double y = 0)
  : roll(r), pitch(p), yaw(y) {}
  explicit EulerAngle(std::string str_rpy, char delimiter = ' ')
  {
    set(str_rpy, delimiter);
  }

  inline void reset()
  {
    set(0, 0, 0);
  }

  inline void set(double r, double p, double y)
  {
    roll = r, pitch = p, yaw = y;
  }

  inline void set(std::string str_rpy, char delimiter = ' ')
  {
    std::vector<std::string> tokens;
    if (StringUtil::split(tokens, str_rpy.c_str(), delimiter) == 3) {
      set(tokens);
    } else {
      throw(myAhrsException("EulerAngle: Invalid String"));
    }
  }

  inline void set(std::vector<std::string> & str_array)
  {
    if (str_array.size() != 3) {
      throw(myAhrsException("EulerAngle: size error"));
    }
    roll = atof(str_array[0].c_str());
    pitch = atof(str_array[1].c_str());
    yaw = atof(str_array[2].c_str());
  }

  inline std::string to_string()
  {
    std::stringstream s;
    s << roll << ", " << pitch << ", " << yaw;
    return s.str();
  }
};

struct DirectionCosineMatrix;
struct Quaternion
{
  double x, y, z, w;

  explicit Quaternion(double _x = 0, double _y = 0, double _z = 0, double _w = 1)
  : x(_x), y(_y), z(_z), w(_w) {}
  explicit Quaternion(std::string str_xyzw, char delimiter = ' ')
  {
    set(str_xyzw, delimiter);
  }

  inline void reset()
  {
    set(0, 0, 0, 1);
  }

  inline void set(double _x, double _y, double _z, double _w)
  {
    x = _x, y = _y, z = _z, w = _w;
  }

  inline void set(std::string str_xyzw, char delimiter = ' ')
  {
    std::vector<std::string> tokens;
    if (StringUtil::split(tokens, str_xyzw.c_str(), delimiter) == 4) {
      set(tokens);
    } else {
      throw(myAhrsException("Quaternion: Invalid String"));
    }
  }

  inline void set(std::vector<std::string> & str_array)
  {
    if (str_array.size() != 4) {
      throw(myAhrsException("Quaternion: size error"));
    }
    x = atof(str_array[0].c_str());
    y = atof(str_array[1].c_str());
    z = atof(str_array[2].c_str());
    w = atof(str_array[3].c_str());
  }

  inline std::string to_string()
  {
    std::stringstream s;
    s << x << ", " << y << ", " << z << ", " << w;
    return s.str();
  }

  void normalize()
  {
    double norm = sqrt(x * x + y * y + z * z + w * w);
    x = x / norm;
    y = y / norm;
    z = z / norm;
    w = w / norm;
  }

  Quaternion conj()
  {
    Quaternion q;
    q.x = -x;
    q.y = -y;
    q.z = -z;
    q.w = w;
    return q;
  }

  // http://www.mathworks.co.kr/kr/help/aerotbx/ug/quatmultiply.html
  // qxr = q*r
  static Quaternion product(Quaternion & q, Quaternion & r)
  {
    Quaternion qxr;

    qxr.w = r.w * q.w - r.x * q.x - r.y * q.y - r.z * q.z;
    qxr.x = r.w * q.x + r.x * q.w - r.y * q.z + r.z * q.y;
    qxr.y = r.w * q.y + r.x * q.z + r.y * q.w - r.z * q.x;
    qxr.z = r.w * q.z - r.x * q.y + r.y * q.x + r.z * q.w;

    return qxr;
  }

  EulerAngle to_euler_angle()
  {
    double xx = x * x;
    double xy = x * y;
    double xz = x * z;
    double xw = x * w;
    double yy = y * y;
    double yz = y * z;
    double yw = y * w;
    double zz = z * z;
    double zw = z * w;
    double ww = w * w;

    double RAD2DEG = 180 / M_PI;
    EulerAngle e;
    e.roll = atan2(2.0 * (yz + xw), -xx - yy + zz + ww) * RAD2DEG;
    e.pitch = -asin(2.0 * (xz - yw)) * RAD2DEG;
    e.yaw = atan2(2.0 * (xy + zw), xx - yy - zz + ww) * RAD2DEG;
    return e;
  }
};

struct DirectionCosineMatrix
{
  double mat[9];

  DirectionCosineMatrix()
  {
    reset();
  }

  explicit DirectionCosineMatrix(double dcm[9])
  {
    set(dcm);
  }

  DirectionCosineMatrix(
    double & m11, double & m12, double & m13,
    double & m21, double & m22, double & m23,
    double & m31, double & m32, double & m33)
  {
    set(m11, m12, m13, m21, m22, m23, m31, m32, m33);
  }

  explicit DirectionCosineMatrix(std::string str_mat, char delimiter = ',')
  {
    set(str_mat, delimiter);
  }

  inline void reset()
  {
    memset(mat, 0, sizeof(mat));
  }

  inline void set(double dcm[9])
  {
    memcpy(mat, dcm, sizeof(mat));
  }

  inline void set(
    double & m11, double & m12, double & m13,
    double & m21, double & m22, double & m23,
    double & m31, double & m32, double & m33)
  {
    mat[0] = m11, mat[1] = m12, mat[2] = m13,
    mat[3] = m21, mat[4] = m22, mat[5] = m23,
    mat[6] = m31, mat[7] = m32, mat[8] = m33;
  }

  inline void set(std::string str_mat, char delimiter = ',')
  {
    std::vector<std::string> tokens;
    if (StringUtil::split(tokens, str_mat.c_str(), delimiter) == 9) {
      set(tokens);
    } else {
      throw(myAhrsException("Matrix3x3: Invalid String"));
    }
  }

  inline void set(std::vector<std::string> & str_array)
  {
    if (str_array.size() != 9) {
      throw(myAhrsException("Matrix3x3: size error"));
    }
    for (int i = 0; i < 9; i++) {
      mat[i] = static_cast<double>(atof(str_array[i].c_str()));
    }
  }

  void set(Quaternion & q)
  {
    // http://www.mathworks.co.kr/kr/help/aeroblks/quaternionstodirectioncosinematrix.html
    double xx = q.x * q.x;
    double xy = q.x * q.y;
    double xz = q.x * q.z;
    double xw = q.x * q.w;
    double yy = q.y * q.y;
    double yz = q.y * q.z;
    double yw = q.y * q.w;
    double zz = q.z * q.z;
    double zw = q.z * q.w;
    double ww = q.w * q.w;

    mat[0] = xx - yy - zz + ww;
    mat[1] = 2.0 * (xy + zw);
    mat[2] = 2.0 * (xz - yw);

    mat[3] = 2.0 * (xy - zw);
    mat[4] = -xx + yy - zz + ww;
    mat[5] = 2.0 * (yz + xw);

    mat[6] = 2.0 * (xz + yw);
    mat[7] = 2.0 * (yz - xw);
    mat[8] = -xx - yy + zz + ww;
  }

  inline std::string to_string()
  {
    std::vector<std::string> temp;
    StringUtil::to_string_list(temp, mat, 9);
    return StringUtil::join(temp, ", ");
  }

  EulerAngle to_euler_angle()
  {
    EulerAngle e;
    double RAD2DEG = 180 / M_PI;
    e.roll = atan2(MAT(1, 2), MAT(2, 2)) * RAD2DEG;
    e.pitch = -asin(MAT(0, 2)) * RAD2DEG;
    e.yaw = atan2(MAT(0, 1), MAT(0, 0)) * RAD2DEG;
    return e;
  }

  Quaternion to_quaternion()
  {
    /*
     *  쿼터니언
     *   - DCM을 기준으로 생각하면 편하므로 DCM으로부터 쿼터니언을 구한다.
     *   - Introduction into quaternions for spacecraft attitude representation
     *      - Dr. -Ing. Zizung Yoon Technical University of Berlin Department of Astronautics and Aeronautics Berlin, Germany
     *      - Eq (6-1)
     *
     */
    double q_w = sqrt((1 + MAT(0, 0) + MAT(1, 1) + MAT(2, 2)) / 4.0);
    double q_x = sqrt((1 + MAT(0, 0) - MAT(1, 1) - MAT(2, 2)) / 4.0);
    double q_y = sqrt((1 - MAT(0, 0) + MAT(1, 1) - MAT(2, 2)) / 4.0);
    double q_z = sqrt((1 - MAT(0, 0) - MAT(1, 1) + MAT(2, 2)) / 4.0);

    double q_list[] = {q_w, q_x, q_y, q_z};
    int max_index = 0;
    double max_value = q_list[0];

    for (int i = 0; i < 4; i++) {
      if (q_list[i] > max_value) {
        max_index = i;
        max_value = q_list[i];
      }
    }

    switch (max_index) {
      case 0:
        q_w = max_value;
        q_x = qvm(3, 2, max_value);
        q_y = qvm(1, 3, max_value);
        q_z = qvm(2, 1, max_value);
        break;

      case 1:
        q_w = qvm(3, 2, max_value);
        q_x = max_value;
        q_y = qvp(2, 1, max_value);
        q_z = qvp(1, 3, max_value);
        break;

      case 2:
        q_w = qvm(1, 3, max_value);
        q_x = qvp(2, 1, max_value);
        q_y = max_value;
        q_z = qvp(3, 2, max_value);
        break;

      case 3:
        q_w = qvm(2, 1, max_value);
        q_x = qvp(1, 3, max_value);
        q_y = qvp(3, 2, max_value);
        q_z = max_value;
        break;
    }

    Quaternion q(q_x, q_y, q_z, q_w);

    return q;
  }

  static void unit_test()
  {
    // http://kr.mathworks.com/help/aerotbx/ug/dcm2quat.html
    //        dcm        = [ 0 1 0; 1 0 0; 0 0 1];
    //        dcm(:,:,2) = [ 0.4330    0.2500   -0.8660; ...
    //                       0.1768    0.9186    0.3536; ...
    //                       0.8839   -0.3062    0.3536];
    //        q = dcm2quat(dcm)
    //
    //        q =
    //
    //            0.7071         0         0         0
    //            0.8224    0.2006    0.5320    0.0223
    const char * dcm_str[] = {
      "0, 1, 0, 1, 0, 0, 0, 0, 1",
      "0.4330, 0.2500, -0.8660, 0.1768, 0.9186, 0.3536, 0.8839, -0.3062, 0.3536",
      "1, 0, 0,  0, -1, 0,  0, 0, -1"};

    for (int i = 0; i < sizeof(dcm_str) / sizeof(dcm_str[0]); i++) {
      DirectionCosineMatrix dcm(dcm_str[i]);
      Quaternion q = dcm.to_quaternion();
      printf("dcm : [%s] \n -> q : %.4f, %.4f, %.4f, %.4f\n", dcm_str[i], q.w, q.x, q.y, q.z);
    }
  }

private:
  inline double MAT(unsigned int row, unsigned int col)
  {
    return mat[(row) * 3 + col];
  }

  double qvm(int row, int col, double denominator)
  {
    return (MAT(row - 1, col - 1) - MAT(col - 1, row - 1)) / (4 * denominator);
  }

  double qvp(int row, int col, double denominator)
  {
    return (MAT(row - 1, col - 1) + MAT(col - 1, row - 1)) / (4 * denominator);
  }
};

/**************************************************************************
 *
 * IMU
 *
 **************************************************************************/
template<typename Type>
struct ImuData
{
  Type ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

  ImuData()
  {
    ax = ay = az = gx = gy = gz = mx = my = mz = temperature = 0;
  }

  explicit ImuData(Type data[10])
  {
    set(data);
  }

  inline void reset()
  {
    Type d[10];
    memset(d, 0, sizeof(d));
    set(d);
  }

  inline void set(Type data[10])
  {
    int i = 0;
    ax = data[i++];
    ay = data[i++];
    az = data[i++];

    gx = data[i++];
    gy = data[i++];
    gz = data[i++];

    mx = data[i++];
    my = data[i++];
    mz = data[i++];

    temperature = data[i++];
  }

  inline void set(std::string str_mat, char delimiter = ' ')
  {
    std::vector<std::string> tokens;
    if (StringUtil::split(tokens, str_mat.c_str(), delimiter) == 10) {
      set(tokens);
    } else {
      throw(myAhrsException("imu: Invalid String"));
    }
  }

  inline void set(std::vector<std::string> & str_array)
  {
    if (str_array.size() != 10) {
      throw(myAhrsException("imu: size error"));
    }
    Type data[10];
    for (int i = 0; i < 10; i++) {
      data[i] = (Type)atof(str_array[i].c_str());
    }
    set(data);
  }

  inline std::string to_string()
  {
    std::stringstream s;
    s << ax << ", " << ay << ", " << az << ", " << gx << ", " << gy << ", " << gz << ", " << mx <<
      ", " << my << ", " << mz << ", " << temperature;
    return s.str();
  }
};

/**************************************************************************
 *
 * AHRS
 *
 **************************************************************************/

enum Attitude
{
  NOT_DEF_ATTITUDE,
  QUATERNION,
  EULER_ANGLE,
};

enum Imu
{
  NOT_DEF_IMU,
  COMPENSATED,
  RAW,
};

struct SensorData
{
  int sequence_number;

  Attitude attitude_type;
  EulerAngle euler_angle;   // degree
  Quaternion quaternion;

  Imu imu_type;
  ImuData<int> imu_rawdata;
  ImuData<float> imu;

  SensorData()
  {
    reset();
  }

  void reset()
  {
    sequence_number = -1;
    euler_angle.reset();
    quaternion.reset();
    imu_rawdata.reset();
    imu.reset();
    attitude_type = NOT_DEF_ATTITUDE;
    imu_type = NOT_DEF_IMU;
  }

  void update_attitude(EulerAngle & e)
  {
    euler_angle = e;
    attitude_type = EULER_ANGLE;
  }

  void update_attitude(Quaternion & q)
  {
    quaternion = q;
    attitude_type = QUATERNION;
  }

  void update_imu(ImuData<int> & i)
  {
    imu_rawdata = i;
    imu_type = RAW;
  }

  void update_imu(ImuData<float> & i)
  {
    imu = i;
    imu_type = COMPENSATED;
  }

  std::string to_string()
  {
    std::vector<std::string> str_array;
    char temp[32];
    snprintf(temp, sizeof(temp), "%d", sequence_number);
    str_array.push_back("sequence = " + std::string(temp));

    switch (attitude_type) {
      case EULER_ANGLE:
        str_array.push_back("euler_angle = " + euler_angle.to_string());
        break;
      case QUATERNION:
        str_array.push_back("quaternion = " + quaternion.to_string());
        break;
      default:
        str_array.push_back("no attitude ");
        break;
    }

    switch (imu_type) {
      case RAW:
        str_array.push_back("imu_raw = " + imu_rawdata.to_string());
        break;
      case COMPENSATED:
        str_array.push_back("imu_comp = " + imu.to_string());
        break;
      default:
        str_array.push_back("no imu ");
        break;
    }
    return StringUtil::join(str_array, "\n");
  }
};

class iMyAhrsPlus
{
  bool debug;

  Platform::SerialPort serial;

  Platform::Mutex mutex_attribute;
  Platform::Mutex mutex_communication;

  bool thread_receiver_ready;
  Platform::Thread thread_receiver;

  bool activate_user_event;
  Platform::Thread thread_event;

  SensorData sensor_data;

  int sensor_id;
  std::map<std::string, std::string> attribute_map;

  typedef bool (iMyAhrsPlus::* AscHandler)(std::vector<std::string> & tokens);
  std::map<std::string, AscHandler> ascii_handler_data_map;

  typedef bool (iMyAhrsPlus::* AscRspHandler)(std::map<std::string, std::string> & attributes);
  std::map<std::string, AscRspHandler> ascii_handler_rsp_map;

  typedef bool (iMyAhrsPlus::* BinHandler)(iNodeParser::Node & node);
  std::map<std::string, BinHandler> binary_handler_data_map;

  static const int ACCEL_RANGE = 16;
  static const int GYRO_RANGE = 2000;
  static const int MAGNET_RANGE = 300;
  static const int TEMP_RANGE = 200;
  static const int EULER_RANGE = 180;

  class Protocol : public iProtocol
  {
    iMyAhrsPlus * ahrs;

public:
    explicit Protocol(iMyAhrsPlus * s)
    : ahrs(s) {}

    void update_attributes(std::vector<std::string> & tokens)
    {
      if (tokens.size() >= 2) {
        ahrs->ascii_parse_response(tokens);
      }
    }

    void update_attributes(std::vector<iNodeParser::Node> & node_list)
    {
      if (node_list.size() > 0) {
        ahrs->binary_parse_response(node_list);
      }
    }
  } protocol;

  class ResponsQueue
  {
    Platform::Mutex lock;
    Platform::Event event;
    std::deque<std::vector<std::string>> queue;

public:
    ResponsQueue() {}
    ~ResponsQueue() {}

    void clear()
    {
      LockGuard _l(lock);
      queue.clear();
    }

    bool wait(int timeout_msec)
    {
      if (size() <= 0) {
        return event.wait(timeout_msec);
      } else {
        return true;
      }
    }

    void push_back(std::vector<std::string> & list)
    {
      LockGuard _l(lock);
      queue.push_back(list);
      event.set();
    }

    size_t size()
    {
      LockGuard _l(lock);
      return queue.size();
    }

    bool pop(std::vector<std::string> & out)
    {
      LockGuard _l(lock);
      if (queue.size() > 0) {
        out = queue.front();
        queue.pop_front();
        return true;
      } else {
        return false;
      }
    }
  } response_message_queue;

  class EventItem
  {
public:
    enum EventId
    {
      NONE = 0,
      EXIT,
      ATTRIBUTE,
      DATA,
    };

    EventId event_id;

    explicit EventItem(EventId id = NONE)
    : event_id(id) {}
    virtual ~EventItem() {}

    virtual SensorData * get_sensor_data() {return 0;}
    virtual std::map<std::string, std::string> * get_attribute() {return 0;}
  };

  class EventQueue
  {
private:
    class EventItemExit : public EventItem
    {
public:
      EventItemExit()
      : EventItem(EXIT) {}
    };

    class EventItemData : public EventItem
    {
      SensorData data;

public:
      explicit EventItemData(SensorData & d)
      : EventItem(DATA), data(d) {}
      ~EventItemData() {}
      SensorData * get_sensor_data() {return &data;}
    };

    class EventItemAttribute : public EventItem
    {
      std::map<std::string, std::string> attribute;

public:
      explicit EventItemAttribute(std::map<std::string, std::string> & a)
      : EventItem(ATTRIBUTE), attribute(a) {}
      ~EventItemAttribute() {}
      std::map<std::string, std::string> * get_attribute() {return &attribute;}
    };

    std::deque<EventItem *> deque;
    Platform::Mutex lock;
    Platform::Event event;

    static const size_t EVENT_MAX_NUM = 5;

public:
    EventQueue() {}
    ~EventQueue()
    {
      int num = deque.size();
      for (int i = 0; i < num; i++) {
        delete pop_event();
      }
    }

    inline bool wait(int timeout_msec = -1)
    {
      return event.wait(timeout_msec);
    }

    void push_event_exit()
    {
      LockGuard _l(lock);
      deque.push_front(new EventItemExit());   // highest priority
      event.set();
    }

    void push_event_attribute_change(std::string & attr_name, std::string & attr_value)
    {
      LockGuard _l(lock);
      std::map<std::string, std::string> attribute;
      attribute[attr_name] = attr_value;
      deque.push_back(new EventItemAttribute(attribute));
      event.set();
    }

    void push_event_data(SensorData & data)
    {
      LockGuard _l(lock);
      if (deque.size() < EVENT_MAX_NUM) {
        deque.push_back(new EventItemData(data));
      }
      event.set();
    }

    EventItem * pop_event()
    {
      LockGuard _l(lock);
      EventItem * i = 0;
      if (deque.size() > 0) {
        i = deque.front();
        deque.pop_front();
      }
      return i;
    }
  } event_queue;

  // prevent copy
  iMyAhrsPlus(iMyAhrsPlus & rhs)
  : protocol(this) {}

public:
  explicit iMyAhrsPlus(std::string port_name = "", unsigned int baudrate = 115200)
  : serial(port_name.c_str(), baudrate), debug(DEBUG_MYAHRS_PLUS), protocol(this), sensor_id(-1),
    activate_user_event(false), thread_receiver_ready(false)
  {
    // response message parser (ascii)
    ascii_handler_rsp_map[std::string("~trig")] = &iMyAhrsPlus::ascii_rsp_trigger;
    ascii_handler_rsp_map[std::string("~ping")] = &iMyAhrsPlus::ascii_rsp_ping;
    ascii_handler_rsp_map[std::string("~divider")] = &iMyAhrsPlus::ascii_rsp_divider;
    ascii_handler_rsp_map[std::string("~mode")] = &iMyAhrsPlus::ascii_rsp_mode;
    ascii_handler_rsp_map[std::string("~asc_out")] = &iMyAhrsPlus::ascii_rsp_asc_out;
    ascii_handler_rsp_map[std::string("~bin_out")] = &iMyAhrsPlus::ascii_rsp_bin_out;
    ascii_handler_rsp_map[std::string("~set_offset")] = &iMyAhrsPlus::ascii_rsp_user_orientation;
    ascii_handler_rsp_map[std::string("~clr_offset")] = &iMyAhrsPlus::ascii_rsp_user_orientation;
    ascii_handler_rsp_map[std::string("~calib")] = &iMyAhrsPlus::ascii_rsp_calib;
    ascii_handler_rsp_map[std::string("~factory")] = &iMyAhrsPlus::ascii_rsp_factory;
    ascii_handler_rsp_map[std::string("~stat")] = &iMyAhrsPlus::ascii_rsp_stat;
    ascii_handler_rsp_map[std::string("~version")] = &iMyAhrsPlus::ascii_rsp_version;
    ascii_handler_rsp_map[std::string("~id")] = &iMyAhrsPlus::ascii_rsp_id;
    ascii_handler_rsp_map[std::string("~sn")] = &iMyAhrsPlus::ascii_rsp_serial_number;
    ascii_handler_rsp_map[std::string("~sensitivity")] = &iMyAhrsPlus::ascii_rsp_sensitivity;
    ascii_handler_rsp_map[std::string("~baudrate")] = &iMyAhrsPlus::ascii_rsp_baudrate;
    ascii_handler_rsp_map[std::string("~save")] = &iMyAhrsPlus::ascii_rsp_save;

    // data message (ascii)
    ascii_handler_data_map[std::string("$RPY")] = &iMyAhrsPlus::ascii_update_euler;
    ascii_handler_data_map[std::string("$QUAT")] = &iMyAhrsPlus::ascii_update_quaternion;
    ascii_handler_data_map[std::string("$RPYIMU")] = &iMyAhrsPlus::ascii_update_rpyimu;
    ascii_handler_data_map[std::string("$QUATIMU")] = &iMyAhrsPlus::ascii_update_quatimu;
    ascii_handler_data_map[std::string("$RIIMU")] = &iMyAhrsPlus::ascii_update_riimu;
    ascii_handler_data_map[std::string("$IMU")] = &iMyAhrsPlus::ascii_update_imu;

    // data node (binary)
    static const char * NAME_DATA_ROOT = "d";
    static const char * NAME_RIIMU = "r";
    static const char * NAME_IMU = "i";
    static const char * NAME_EULER = "e";
    static const char * NAME_QUATERNION = "q";
    static const char * NAME_DCM = "c";
    static const char * NAME_SEQUANCE = "s";

    binary_handler_data_map[std::string(NAME_SEQUANCE)] = &iMyAhrsPlus::binary_update_sequence;
    binary_handler_data_map[std::string(NAME_EULER)] = &iMyAhrsPlus::binary_update_euler;
    binary_handler_data_map[std::string(NAME_QUATERNION)] = &iMyAhrsPlus::binary_update_quaternion;
    binary_handler_data_map[std::string(NAME_IMU)] = &iMyAhrsPlus::binary_update_imu;
    binary_handler_data_map[std::string(NAME_RIIMU)] = &iMyAhrsPlus::binary_update_riimu;

    thread_event.start(thread_proc_callback, reinterpret_cast<void *>(this));
  }

  virtual ~iMyAhrsPlus()
  {
    event_queue.push_event_exit();
    stop();
    thread_event.join();
  }

  const char * sdk_version()
  {
    return WITHROBOT_MYAHRS_PLUS_SDK_VERSION;
  }

  bool start(std::string port_name = "", int baudrate = -1)
  {
    {
      LockGuard _l(mutex_communication);
      if (port_name == "" || baudrate < 0) {
        if (serial.Open() == false) {
          return false;
        }
      } else {
        if (serial.Open(port_name.c_str(), baudrate) == false) {
          return false;
        }
      }

      serial.Flush();

      thread_receiver_ready = false;
      if (thread_receiver.start(thread_proc_receiver, reinterpret_cast<void *>(this)) == false) {
        return false;
      }

      while (thread_receiver_ready == false) {
        Platform::msleep(10);
      }
    }

    for (int i = 0; i < 3; i++) {
      cmd_ping(1000);
    }

    activate_user_event = resync();
    return activate_user_event;
  }

  void stop()
  {
    LockGuard _l(mutex_communication);
    clear_all_attribute();
    activate_user_event = false;
    serial.Close();
    thread_receiver.join();
  }

  inline int get_sensor_id()
  {
    LockGuard _l(mutex_attribute);
    return sensor_id;
  }

  inline SensorData get_data()
  {
    LockGuard _l(mutex_attribute);
    return sensor_data;
  }

  inline void get_data(SensorData & data)
  {
    LockGuard _l(mutex_attribute);
    data = sensor_data;
  }

  /*
   *  access attributes
   */
  bool get_attribute(const char * attrib_name, std::string & attrib_value)
  {
    LockGuard _l(mutex_attribute);

    std::string attribute_name(attrib_name);
    std::map<std::string, std::string>::iterator it = attribute_map.find(attribute_name);
    if (it != attribute_map.end()) {
      attrib_value = it->second;
      return true;
    } else {
      return false;
    }
  }

private:
  bool is_exist(const std::string & attribute_name)
  {
    std::map<std::string, std::string>::iterator it = attribute_map.find(attribute_name);
    if (it != attribute_map.end()) {
      return true;
    } else {
      return false;
    }
  }

  void set_attribute(const std::string & attribute_name, const std::string & value)
  {
    attribute_map[attribute_name] = value;

    if (activate_user_event) {
      std::string attr_name = attribute_name;
      std::string attr_value = value;
      event_queue.push_event_attribute_change(attr_name, attr_value);
    }
  }

  void clear_all_attribute()
  {
    LockGuard _l(mutex_attribute);
    attribute_map.clear();
    sensor_id = -1;
  }

public:
  // ########################################################################################################################
  // #  accel_bias : -1.334012e+01, 2.941270e+01, 1.579460e+02
  // #  accel_max : 16
  // #  accel_sensitivity : 4.882813e-04
  // #  accel_sensitivity_matrix :
  // #       4.888283e-04, -1.201399e-06, -4.818384e-06,
  // #       0.000000e+00,  4.901073e-04, -1.257056e-06,
  // #       0.000000e+00,  0.000000e+00,  4.853439e-04
  // #  ascii_format : QUATIMU
  // #  baudrate : 115200
  // #  binary_format : QUATERNION, IMU
  // #  build_date : Jul 12 2014 18:55:53
  // #  coordinate_transformation_global_to_user :
  // #       0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00
  // #  coordinate_transformation_sensor_to_vehicle :
  // #       0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00
  // #  divider : 1
  // #  firmware_version : 1.5
  // #  gyro_bias : -1.633780e+01, -7.488200e+00, -1.367940e+01
  // #  gyro_max : 2000
  // #  gyro_sensitivity : 6.097561e-02
  // #  gyro_sensitivity_matrix :
  // #       6.128651e-02, -2.204396e-04, -9.754782e-04,
  // #       1.441085e-05,  6.082033e-02,  5.099393e-04,
  // #      -1.803549e-04,  7.779496e-04,  6.148182e-02
  // #  magnet_bias : 9.936985e+01, 1.039952e+01, -9.331067e+01
  // #  magnet_sensitivity_matrix :
  // #       6.128308e-01, 1.720049e-03, 9.577197e-03,
  // #       1.720049e-03, 5.859380e-01, 1.020694e-03,
  // #       9.577197e-03, 1.020694e-03, 6.279327e-01
  // #  max_rate : 100
  // #  mode : BT
  // #  platform : myAhrsRevC
  // #  product_name : myAHRS+
  // #  sensor_id : 0
  // #  sensor_serial_number : 464432970808430886
  // #  yaw_offset : OFF
  // ########################################################################################################################

  std::vector<std::string> get_attribute_list()
  {
    LockGuard _l(mutex_attribute);
    std::vector<std::string> attribute_list;
    for (std::map<std::string, std::string>::iterator it = attribute_map.begin();
      it != attribute_map.end(); it++)
    {
      attribute_list.push_back(it->first);
    }
    return attribute_list;
  }

  /*
   * re-sync all attributes
   */
  bool resync()
  {
    bool ok = false;
    std::string mode;

    do {
      if (cmd_mode() == false) {
        DBG_PRINTF(true, "cmd_mode() returns false\n");
        break;
      }
      if (get_attribute("mode", mode) == false) {
        DBG_PRINTF(true, "get_attribute('mode') returns false\n");
        break;
      }
      if (cmd_mode("T") == false) {
        DBG_PRINTF(true, "cmd_mode(T) returns false\n");
        break;
      }
      if (cmd_divider() == false) {
        DBG_PRINTF(true, "cmd_divider() returns false\n");
        break;
      }
      if (cmd_ascii_data_format() == false) {
        DBG_PRINTF(true, "cmd_ascii_data_format() returns false\n");
        break;
      }
      if (cmd_binary_data_format() == false) {
        DBG_PRINTF(true, "cmd_binary_data_format() returns false\n");
        break;
      }
      if (cmd_set_user_orientation_offset() == false) {
        DBG_PRINTF(true, "cmd_set_user_orientation_offset() returns false\n");
        break;
      }
      if (cmd_calibration_parameter('A') == false) {
        DBG_PRINTF(true, "cmd_calibration_parameter(A) returns false\n");
        break;
      }
      if (cmd_calibration_parameter('G') == false) {
        DBG_PRINTF(true, "cmd_calibration_parameter(G) returns false\n");
        break;
      }
      if (cmd_calibration_parameter('M') == false) {
        DBG_PRINTF(true, "cmd_calibration_parameter(M) returns false\n");
        break;
      }
      if (cmd_version() == false) {
        DBG_PRINTF(true, "cmd_version() returns false\n");
        break;
      }
      if (cmd_id() == false) {
        DBG_PRINTF(true, "cmd_id() returns false\n");
        break;
      }
      if (cmd_sensitivity() == false) {
        DBG_PRINTF(true, "cmd_sensitivity() returns false\n");
        break;
      }
      if (cmd_baudrate() == false) {
        DBG_PRINTF(true, "cmd_baudrate() returns false\n");
        break;
      }
      if (cmd_mode(mode.c_str()) == false) {
        DBG_PRINTF(true, "cmd_mode(restore) returns false\n");
        break;
      }

      ok = true;
    } while (0);

    if (ok == true && debug) {
      DBG_PRINTF(debug, "\n\n### ATTRIBUTES #####\n");
      for (std::map<std::string, std::string>::iterator it = attribute_map.begin();
        it != attribute_map.end(); ++it)
      {
        DBG_PRINTF(debug, "\t- attribute(%s) = \"%s\"\n", it->first.c_str(), it->second.c_str());
      }
    }

    if (debug && ok == false) {
      Platform::msleep(100);
    }

    return ok;
  }

  /*
   *  myahrs_plus commands
   */
  void cmd_trigger()
  {
    // no response, no wait
    send_command("@trig", -1);
  }

  bool cmd_ping(int timeout_msec = 500)
  {
    return send_command("@ping", timeout_msec);
  }

  bool cmd_divider(int timeout_msec = 500)
  {
    return send_command("@divider", timeout_msec);
  }

  bool cmd_divider(const char * divider, int timeout_msec = 500)
  {
    if (strlen(divider) > 100) {
      return false;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "@divider,%s", divider);
    return send_command(buf, timeout_msec);
  }

  bool cmd_mode(const char * mode_string = 0, int timeout_msec = 500)
  {
    if (mode_string == 0) {
      return send_command("@mode", timeout_msec);
    } else {
      std::string command = std::string("@mode,") + std::string(mode_string);
      return send_command(command.c_str(), timeout_msec);
    }
  }

  bool cmd_ascii_data_format(const char * asc_output = 0, int timeout_msec = 500)
  {
    if (asc_output == 0) {
      return send_command("@asc_out", timeout_msec);
    } else {
      std::string command = std::string("@asc_out,") + std::string(asc_output);
      return send_command(command.c_str(), timeout_msec);
    }
  }

  bool cmd_binary_data_format(const char * bin_output = 0, int timeout_msec = 500)
  {
    if (bin_output == 0) {
      return send_command("@bin_out", timeout_msec);
    } else {
      std::string command = std::string("@bin_out,") + std::string(bin_output);
      return send_command(command.c_str(), timeout_msec);
    }
  }

  bool cmd_set_user_orientation_offset(int timeout_msec = 500)
  {
    return send_command("@set_offset", timeout_msec);
  }

  bool cmd_set_user_orientation_offset(const char * enable_yaw_offset, int timeout_msec = 500)
  {
    std::string command = std::string("@set_offset,") + std::string(enable_yaw_offset);
    return send_command(command.c_str(), timeout_msec);
  }

  bool cmd_clear_user_orientation_offset(int timeout_msec = 500)
  {
    return send_command("@clr_offset", timeout_msec);
  }

  bool cmd_calibration_parameter(
    char sensor_type, const char * calibration_parameters = 0,
    int timeout_msec = 500)
  {
    char buf[512];
    if (calibration_parameters == 0) {
      snprintf(buf, sizeof(buf), "@calib,%c", sensor_type);
      return send_command(buf, timeout_msec);
    } else {
      if (strlen(calibration_parameters) > sizeof(buf) - 20) {
        return false;
      }
      snprintf(buf, sizeof(buf), "@calib,%c,%s", sensor_type, calibration_parameters);
      return send_command(buf, timeout_msec);
    }
  }

  bool cmd_restore_all_default(int timeout_msec = 500)
  {
    return send_command("@factory", timeout_msec);
  }

  bool cmd_version(int timeout_msec = 500)
  {
    return send_command("@version", timeout_msec);
  }

  bool cmd_id(int timeout_msec = 500)
  {
    return send_command("@id", timeout_msec);
  }

  bool cmd_id(const char * str_sensor_id, int timeout_msec = 500)
  {
    if (strlen(str_sensor_id) > 100) {
      return false;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "@id,%s", str_sensor_id);
    return send_command(buf, timeout_msec);
  }

  bool cmd_serial_number(int timeout_msec = 500)
  {
    return send_command("@sn", timeout_msec);
  }

  bool cmd_sensitivity(int timeout_msec = 500)
  {
    return send_command("@sensitivity", timeout_msec);
  }

  bool cmd_baudrate(int timeout_msec = 500)
  {
    return send_command("@baudrate", timeout_msec);
  }

  bool cmd_baudrate(const char * baudrate, int timeout_msec = 500)
  {
    if (strlen(baudrate) > 100) {
      return false;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "@baudrate,%s", baudrate);
    return send_command(buf, timeout_msec);
  }

  bool cmd_save(int timeout_msec = 500)
  {
    return send_command("@save", timeout_msec);
  }

protected:
  /*
   *  Event handler interface
   */
  virtual void OnSensorData(int sensor_id, SensorData sensor_data) {}
  virtual void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value) {}

private:
  bool send_command(std::string command_string, int timeout_msec)
  {
    LockGuard _l(mutex_communication);

    DBG_PRINTF(debug, "### SEND : %s\n", command_string.c_str());

    uint8_t crc = 0;
    char crc_string[16];
    for (size_t i = 0; i < command_string.length(); i++) {
      crc ^= command_string[i];
    }
    snprintf(crc_string, sizeof(crc_string), "*%02X\r\n", crc);
    std::string command_with_crc = command_string + std::string(crc_string);

    /*
     * send command
     */
    response_message_queue.clear();
    if (serial.Write((unsigned char *)command_with_crc.c_str(), command_with_crc.length()) <= 0) {
      DBG_PRINTF(debug, "serial.Write() return false\n");
      return false;
    }

    if (timeout_msec < 0) {
      // no wait
      return true;
    }

    std::vector<std::string> tokens;
    std::vector<std::string> cmd_tokens;

    do {
      /*
       * wait for response
       */
      DBG_PRINTF(debug, "### WAITING RESPONSE\n");
      if (response_message_queue.wait(timeout_msec) == false) {
        DBG_PRINTF(debug, "TIMEOUT!!(%d)\n", timeout_msec);
        return false;
      }

#ifdef WIN32
      /*
       * stupid windows...
       */
      while (response_message_queue.size() == 0) {
      }
#endif  // WIN32

      /*
       *  check response
       */
      tokens.clear();
      if (response_message_queue.pop(tokens) == false) {
        if (debug) {
          Platform::msleep(10);
          DBG_PRINTF(
            debug,
            "ERROR??? : response_queue.pop() returns false (response_queue.size() %lu, cmd=%s)\n",
            response_message_queue.size(), command_string.c_str());
        }
        return false;
      }

      cmd_tokens.clear();
      StringUtil::split(cmd_tokens, command_string.c_str(), ',');
      StringUtil::replace(cmd_tokens[0], "@", "~");
      if (tokens[0] == cmd_tokens[0]) {
        DBG_PRINTF(debug, "### RCV OK(%s)\n", cmd_tokens[0].c_str());
        break;
      } else {
        DBG_PRINTF(
          debug, "ERROR: invalid response. command %s, response %s)\n",
          cmd_tokens[0].c_str(), tokens[0].c_str());
        continue;
      }
    } while (1);

    /*
     *  handle error response
     */
    if (tokens[1] != std::string("OK")) {
      // print error message
      DBG_PRINTF(debug, "ERROR: status is not OK. command %s)\n", cmd_tokens[0].c_str());
      return false;
    }

    /*
     *  run response handler
     */
    std::map<std::string, AscRspHandler>::iterator it = ascii_handler_rsp_map.find(tokens[0]);
    if (it != ascii_handler_rsp_map.end()) {
      std::map<std::string, std::string> attributes;
      bool res = true;
      if (StringUtil::extract_attributes(attributes, tokens) > 0) {
        LockGuard _l(mutex_attribute);
        res = (this->*(it->second))(attributes);
        if (res == false) {
          DBG_PRINTF(
            debug, "ERROR: message hander returns false. command %s)\n",
            cmd_tokens[0].c_str());
        }
      }

      if (res) {
        DBG_PRINTF(
          debug, "### OK : message hander returns true. command %s, rcvq_sz %d)\n",
          cmd_tokens[0].c_str(), response_message_queue.size());
      }

      return res;
    } else {
      return false;
    }
  }

  /*
   * threads
   */
  void proc_receiver()
  {
    int len;
    unsigned char buffer[1024];

    thread_receiver_ready = true;

    while (true) {
      memset(buffer, 0, sizeof(buffer));
      len = serial.Read(buffer, sizeof(buffer) - 1);
      if (len == 0) {
        Platform::msleep(1);
        continue;
      } else if (len < 0) {
        break;   // stop thread
      } else if (len > 0) {
        DBG_PRINTF(debug, "### SZ(%d) [%s]\n", len, buffer);
        protocol.feed(buffer, len);
      }
    }

    DBG_PRINTF(debug, "### %s() exit\n", __FUNCTION__);
  }

  void proc_callback()
  {
    EventItem * event;
    bool exit_thread = false;

    while (exit_thread == false) {
      event = event_queue.pop_event();
      if (event == 0) {
        event_queue.wait();
        continue;
      }

      try {
        switch (event->event_id) {
          case EventItem::EXIT:
            DBG_PRINTF(debug, "receive EventItem::EXIT\n");
            exit_thread = true;
            break;
          case EventItem::ATTRIBUTE:
            {
              std::map<std::string, std::string> * attribute_pair = event->get_attribute();
              for (std::map<std::string, std::string>::iterator it = attribute_pair->begin();
                it != attribute_pair->end(); it++)
              {
                OnAttributeChange(sensor_id, it->first, it->second);
              }
            }
            break;
          case EventItem::DATA:
            {
              SensorData * sensor_data = event->get_sensor_data();
              OnSensorData(sensor_id, *sensor_data);
            }
            break;
          default:
            break;
        }
      } catch (...) {
      }

      delete event;
    }   // while(exit_thread == false)

    DBG_PRINTF(debug, "### %s() exit\n", __FUNCTION__);
  }

  static void * thread_proc_receiver(void * arg)
  {
    reinterpret_cast<iMyAhrsPlus *>(arg)->proc_receiver();
    return 0;
  }

  static void * thread_proc_callback(void * arg)
  {
    reinterpret_cast<iMyAhrsPlus *>(arg)->proc_callback();
    return 0;
  }

  /*
   *  ascii response handlers
   */
  bool ascii_parse_response(std::vector<std::string> & tokens)
  {
    if (tokens[0][0] == iAsciiProtocol::MSG_HDR_RESPONSE) {
      // message parsing will be delegated to send_command()
      response_message_queue.push_back(tokens);
      return true;
    } else {
      LockGuard _l(mutex_attribute);

      sensor_data.reset();
      bool res = false;

      std::map<std::string, AscHandler>::iterator it = ascii_handler_data_map.find(tokens[0]);
      if (it != ascii_handler_data_map.end()) {
        res = (this->*(it->second))(tokens);
      }

      if (res && activate_user_event) {
        event_queue.push_event_data(sensor_data);
      }

      return res;
    }
  }

  void dbg_show_all_attributes(std::map<std::string, std::string> & attributes)
  {
    for (std::map<std::string, std::string>::iterator it = attributes.begin();
      it != attributes.end(); ++it)
    {
      DBG_PRINTF(debug, "   --- %s : %s\n", (it->first).c_str(), (it->second).c_str());
    }
  }

  bool ascii_rsp_trigger(std::map<std::string, std::string> & attributes)
  {
    return true;
  }

  bool ascii_rsp_ping(std::map<std::string, std::string> & attributes)
  {
    return true;
  }

  bool ascii_rsp_divider(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("divider", attributes["divider"]);
    set_attribute("max_rate", attributes["max_rate"]);

    return true;
  }

  bool ascii_rsp_mode(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("mode", attributes["mode"]);

    return true;
  }

  bool ascii_rsp_asc_out(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("ascii_format", attributes["fmt"]);

    return true;
  }

  bool ascii_rsp_bin_out(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    std::vector<std::string> prop_bin_format;
    StringUtil::split(prop_bin_format, attributes["fmt"].c_str(), ' ');
    set_attribute("binary_format", StringUtil::join(prop_bin_format, ", "));

    return true;
  }

  bool ascii_rsp_user_orientation(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    // ~set_offset,OK,yaw_offset=OFF,
    //       q_s2v=0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00,
    //       q_g2u=0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00*12

    set_attribute("yaw_offset", attributes["yaw_offset"]);

    std::vector<std::string> parameters;

    if (StringUtil::split(parameters, attributes["q_s2v"].c_str(), ' ') != 4) {
      return false;
    }
    set_attribute(
      "coordinate_transformation_sensor_to_vehicle",
      StringUtil::join(parameters, ", "));

    if (StringUtil::split(parameters, attributes["q_g2u"].c_str(), ' ') != 4) {
      return false;
    }
    set_attribute("coordinate_transformation_global_to_user", StringUtil::join(parameters, ", "));

    return true;
  }

  bool ascii_rsp_calib(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    // ~calib,OK,sensor=A,
    //      param=4.882813e-04 0.000000e+00 0.000000e+00 0.000000e+00
    //            4.882813e-04 0.000000e+00 0.000000e+00 0.000000e+00
    //            4.882813e-04 0.000000e+00 0.000000e+00 0.000000e+00*0E

    std::string sensor = attributes["sensor"];

    switch (static_cast<char>(sensor[0])) {
      case 'A':
        sensor = "accel";
        break;
      case 'G':
        sensor = "gyro";
        break;
      case 'M':
        sensor = "magnet";
        break;
      default:
        return false;
    }

    std::vector<std::string> parameters;
    if (StringUtil::split(parameters, attributes["param"].c_str(), ' ') != 12) {
      return false;
    }

    std::vector<std::string> transform(9, "");
    std::vector<std::string> bias(3, "");

    std::copy_n(parameters.begin(), 9, transform.begin());
    std::copy_n(parameters.begin() + 9, 3, bias.begin());

    set_attribute(sensor + "_calibration_matrix", StringUtil::join(transform, ", "));
    set_attribute(sensor + "_bias", StringUtil::join(bias, ", "));

    return true;
  }

  bool ascii_rsp_factory(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    return true;
  }

  bool ascii_rsp_stat(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    return true;
  }

  bool ascii_rsp_version(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("build_date", attributes["build"]);
    set_attribute("platform", attributes["platform"]);
    set_attribute("product_name", attributes["product"]);
    set_attribute("sensor_serial_number", attributes["sn"]);
    set_attribute("firmware_version", attributes["ver"]);

    return true;
  }

  bool ascii_rsp_id(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    std::string str_sensor_id = attributes["id"];
    set_attribute("sensor_id", str_sensor_id);
    sensor_id = atoi(str_sensor_id.c_str());

    return true;
  }

  bool ascii_rsp_serial_number(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("sensor_serial_number", attributes["sn"]);

    return true;
  }

  bool ascii_rsp_sensitivity(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("accel_max", attributes["acc_range"]);
    set_attribute("gyro_max", attributes["gyro_range"]);
    set_attribute("accel_sensitivity", attributes["acc_sensitivity"]);
    set_attribute("gyro_sensitivity", attributes["gyro_sensitivity"]);

    return true;
  }

  bool ascii_rsp_baudrate(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    set_attribute("baudrate", attributes["baudrate"]);

    return true;
  }

  bool ascii_rsp_save(std::map<std::string, std::string> & attributes)
  {
    dbg_show_all_attributes(attributes);

    return true;
  }

  /*
   *  ascii data message handlers
   */

  // $RPY,04,-1.55,-1.25,96.94*50
  bool ascii_update_euler(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM = 3;
    if (tokens.size() != DATA_NUM + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_euler(DATA_NUM, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM, str_euler.begin());

    EulerAngle e;
    e.set(str_euler);
    sensor_data.update_attitude(e);

    DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_euler, ", ").c_str());

    return true;
  }

  // $QUAT,68,0.0006,0.0174,-0.7489,-0.6625*16
  bool ascii_update_quaternion(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM = 4;
    if (tokens.size() != DATA_NUM + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_quat(DATA_NUM, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM, str_quat.begin());

    Quaternion q;
    q.set(str_quat);
    sensor_data.update_attitude(q);

    DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_quat, ", ").c_str());

    return true;
  }

  // $RPYIMU,15,-1.55,-1.25,97.31,-0.0142,-0.0010,-0.9224,-0.9756,-0.3659,-0.8537,-8.4000,-46.8000,5.4000,38.3*36
  bool ascii_update_rpyimu(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM_EULER = 3;
    static const int DATA_NUM_IMU = 10;
    if (tokens.size() != DATA_NUM_EULER + DATA_NUM_IMU + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_euler(DATA_NUM_EULER, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM_EULER, str_euler.begin());

    EulerAngle e;
    e.set(str_euler);
    sensor_data.update_attitude(e);

    std::vector<std::string> str_imu(DATA_NUM_IMU, "");
    std::copy_n(tokens.begin() + 2 + DATA_NUM_EULER, DATA_NUM_IMU, str_imu.begin());

    ImuData<float> imu;
    imu.set(str_imu);
    sensor_data.update_imu(imu);

    DBG_PRINTF(
      debug, "### %s(euler=(%s), imu=(%s))\n", __FUNCTION__,
      StringUtil::join(str_euler, ", ").c_str(), StringUtil::join(str_imu, ", ").c_str());

    return true;
  }

  // $QUATIMU,53,0.0424,-0.1791,0.2366,0.9540,-0.3636,0.0027,-0.9260,0.0156,0.1537,0.2896,212.2648,-72.7573,168.2144,36.8*7F
  bool ascii_update_quatimu(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM_QUAT = 4;
    static const int DATA_NUM_IMU = 10;
    if (tokens.size() != DATA_NUM_QUAT + DATA_NUM_IMU + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_quat(DATA_NUM_QUAT, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM_QUAT, str_quat.begin());

    Quaternion q;
    q.set(str_quat);
    sensor_data.update_attitude(q);

    std::vector<std::string> str_imu(DATA_NUM_IMU, "");
    std::copy_n(tokens.begin() + 2 + DATA_NUM_QUAT, DATA_NUM_IMU, str_imu.begin());

    ImuData<float> imu;
    imu.set(str_imu);
    sensor_data.update_imu(imu);

    DBG_PRINTF(
      debug, "### %s(quaternion=(%s), imu=(%s))\n", __FUNCTION__,
      StringUtil::join(str_quat, ", ").c_str(), StringUtil::join(str_imu, ", ").c_str());

    return true;
  }

  // $RIIMU,59,-16,-8,-1897,-14,-7,-12,-26,-156,18,1101*79
  bool ascii_update_riimu(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM = 10;
    if (tokens.size() != DATA_NUM + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_imu(DATA_NUM, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM, str_imu.begin());

    ImuData<int> imu_rawdata;
    imu_rawdata.set(str_imu);
    sensor_data.update_imu(imu_rawdata);

    DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_imu, ", ").c_str());

    return true;
  }

  // $IMU,74,-0.0054,-0.0015,-0.9204,-0.7317,-0.4878,-0.7317,-7.2000,-45.6000,6.6000,38.2*68
  bool ascii_update_imu(std::vector<std::string> & tokens)
  {
    static const int DATA_NUM = 10;
    if (tokens.size() != DATA_NUM + 2) {
      return false;
    }

    sensor_data.sequence_number = atoi(tokens[1].c_str());

    std::vector<std::string> str_imu(DATA_NUM, "");
    std::copy_n(tokens.begin() + 2, DATA_NUM, str_imu.begin());

    ImuData<float> imu;
    imu.set(str_imu);
    sensor_data.update_imu(imu);

    DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_imu, ", ").c_str());

    return true;
  }

  /*
   * binary data message handlers
   */
  bool binary_parse_response(std::vector<iNodeParser::Node> & node_list)
  {
    LockGuard _l(mutex_attribute);

    sensor_data.reset();

    if (debug) {
      for (size_t i = 0; i < node_list.size(); i++) {
        DBG_PRINTF(
          debug, "\t#### node(%lu/%lu)->name[%02X] : %s\n", i, node_list.size(),
          node_list[i].name[0], node_list[i].name.c_str());
      }
    }

    for (size_t i = 0; i < node_list.size(); i++) {
      iNodeParser::Node & node = node_list[i];
      std::map<std::string, BinHandler>::iterator it = binary_handler_data_map.find(node.name);
      if (it != binary_handler_data_map.end()) {
        if ((this->*(it->second))(node) == false) {
          return false;
        }
      }
    }

    if (activate_user_event) {
      event_queue.push_event_data(sensor_data);
    }

    return true;
  }

  float int16tofloat(int16_t value, float value_max)
  {
    static const int SCALE_FACTOR = 0x7FFF;
    double ratio = static_cast<double>(value) / SCALE_FACTOR;
    return static_cast<float>(ratio * value_max);
  }

  bool binary_update_sequence(iNodeParser::Node & node)
  {
    sensor_data.sequence_number = node.list[0].value.ui8;

    DBG_PRINTF(debug, "### binary_update_sequence(%d)\n", node.list[0].value.ui8);

    return true;
  }

  bool binary_update_euler(iNodeParser::Node & node)
  {
    static const int DATA_NUM = 3;
    if (node.list.size() != DATA_NUM) {
      return false;
    }

    float roll = int16tofloat(node.list[0].value.i16, static_cast<float>(EULER_RANGE));
    float pitch = int16tofloat(node.list[1].value.i16, static_cast<float>(EULER_RANGE));
    float yaw = int16tofloat(node.list[2].value.i16, static_cast<float>(EULER_RANGE));

    EulerAngle e;
    e.set(roll, pitch, yaw);
    sensor_data.update_attitude(e);

    DBG_PRINTF(debug, "### %s(%f, %f, %f)\n", __FUNCTION__, roll, pitch, yaw);

    return true;
  }

  bool binary_update_quaternion(iNodeParser::Node & node)
  {
    static const int DATA_NUM = 4;
    if (node.list.size() != DATA_NUM) {
      return false;
    }

    float x = int16tofloat(node.list[0].value.i16, 1);
    float y = int16tofloat(node.list[1].value.i16, 1);
    float z = int16tofloat(node.list[2].value.i16, 1);
    float w = int16tofloat(node.list[3].value.i16, 1);

    Quaternion q;
    q.set(x, y, z, w);
    sensor_data.update_attitude(q);

    DBG_PRINTF(debug, "### %s(%f, %f, %f, %f)\n", __FUNCTION__, x, y, z, w);

    return true;
  }

  bool binary_update_imu(iNodeParser::Node & node)
  {
    static const int DATA_NUM = 10;
    if (node.list.size() != DATA_NUM) {
      return false;
    }

    float m[DATA_NUM];

    m[0] = int16tofloat(node.list[0].value.i16, static_cast<float>(ACCEL_RANGE));
    m[1] = int16tofloat(node.list[1].value.i16, static_cast<float>(ACCEL_RANGE));
    m[2] = int16tofloat(node.list[2].value.i16, static_cast<float>(ACCEL_RANGE));

    m[3] = int16tofloat(node.list[3].value.i16, static_cast<float>(GYRO_RANGE));
    m[4] = int16tofloat(node.list[4].value.i16, static_cast<float>(GYRO_RANGE));
    m[5] = int16tofloat(node.list[5].value.i16, static_cast<float>(GYRO_RANGE));

    m[6] = int16tofloat(node.list[6].value.i16, static_cast<float>(MAGNET_RANGE));
    m[7] = int16tofloat(node.list[7].value.i16, static_cast<float>(MAGNET_RANGE));
    m[8] = int16tofloat(node.list[8].value.i16, static_cast<float>(MAGNET_RANGE));

    m[9] = int16tofloat(node.list[9].value.i16, static_cast<float>(TEMP_RANGE));

    ImuData<float> imu;
    imu.set(m);
    sensor_data.update_imu(imu);

    if (debug) {
      std::vector<std::string> str_list;
      StringUtil::to_string_list(str_list, m, DATA_NUM);
      DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_list, ", ").c_str());
    }

    return true;
  }

  bool binary_update_riimu(iNodeParser::Node & node)
  {
    static const int DATA_NUM = 10;
    if (node.list.size() != DATA_NUM) {
      return false;
    }

    int m[DATA_NUM];
    for (int i = 0; i < DATA_NUM; i++) {
      m[i] = node.list[i].value.i16;
    }

    ImuData<int> imu_rawdata;
    imu_rawdata.set(m);
    sensor_data.update_imu(imu_rawdata);

    if (debug) {
      std::vector<std::string> str_list;
      StringUtil::to_string_list(str_list, m, DATA_NUM);
      DBG_PRINTF(debug, "### %s(%s)\n", __FUNCTION__, StringUtil::join(str_list, ", ").c_str());
    }

    return true;
  }
};   // iMyAhrsPlus

class MyAhrsPlus : public iMyAhrsPlus
{
  Platform::Event event;
  Platform::Mutex lock;

  void (* attribute_callback)(
    void * context, int sensor_id, const char * attribute_name,
    const char * value);
  void * attribute_callback_context;

  void (* data_callback)(void * context, int sensor_id, SensorData * sensor_data);
  void * data_callback_context;

  uint32_t sample_count;

public:
  explicit MyAhrsPlus(std::string port = "", unsigned int baudrate = 115200)
  : iMyAhrsPlus(port, baudrate), sample_count(0), attribute_callback(0), attribute_callback_context(
      0), data_callback(0), data_callback_context(0)
  {
  }

  virtual ~MyAhrsPlus()
  {
    attribute_callback = 0;
    attribute_callback_context = 0;
    data_callback = 0;
    data_callback_context = 0;
  }

  bool wait_data(int timeout_msec = 500)
  {
    return event.wait(timeout_msec);
  }

  void register_attribute_callback(
    void (* callback)(
      void * context, int sensor_id,
      const char * attribute_name,
      const char * value), void * callback_context)
  {
    LockGuard _l(lock);
    attribute_callback_context = callback_context;
    attribute_callback = callback;
  }

  void register_data_callback(
    void (* callback)(
      void * context, int sensor_id,
      SensorData * sensor_data), void * callback_context)
  {
    LockGuard _l(lock);
    data_callback_context = callback_context;
    data_callback = callback;
  }

  inline uint32_t get_sample_count()
  {
    LockGuard _l(lock);
    return sample_count;
  }

protected:
  void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
  {
    LockGuard _l(lock);
    if (attribute_callback) {
      try {
        attribute_callback(
          attribute_callback_context, sensor_id,
          attribute_name.c_str(), value.c_str());
      } catch (...) {
      }
    }
  }

  void OnSensorData(int sensor_id, SensorData data)
  {
    LockGuard _l(lock);
    sample_count++;
    event.set();
    if (data_callback) {
      try {
        data_callback(data_callback_context, sensor_id, &data);
      } catch (...) {
      }
    }
  }
};
}  // namespace WithRobot

#endif  // MYAHRS_ROS2_DRIVER__MYAHRS_PLUS_HPP_
