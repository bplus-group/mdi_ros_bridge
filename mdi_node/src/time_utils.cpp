// MIT License
//
// Copyright (c) 2022 b-plus technologies GmbH
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "mdi_publisher.h"

/*
    Some auxillary functions to handle the node similar to what we provide with the samples of the MDI RX library.

    Additionally, we load the MDI RX Library dynamically! The library is distributed as a statically linked (for linux
    linked against libc of gcc 4.8 or so) package without any installer to bundle your application with the version you need.
    Therefore we deploy the library along with our node, but we need to manually load it. 

    Right now, only support ROS2 on linux, but Windows would also be possible.
*/


#ifdef WIN32
    #include <fileapi.h>
    inline int __eventWait(HANDLE h, uint32_t timeout) {
    uint32_t x = WaitForSingleObject(h, timeout);
    return x==WAIT_OBJECT_0;
    }

    uint64_t CreateTimestampUs() {
    uint64_t t, f;
    QueryPerformanceCounter((PLARGE_INTEGER)&t);
    QueryPerformanceFrequency((PLARGE_INTEGER)&f);
    return (((uint64_t)(t)) * ((uint64_t)1000000)) / ((uint64_t) f);
    }
#else 
    #include <dlfcn.h>
    #include <unistd.h>
    #include <limits.h>
    #include <sys/select.h>

    int __eventWait(int fd, uint32_t timeout) {
    int result = 3;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    timeval tv;
    tv.tv_sec = (int)(timeout / 1000);
    tv.tv_usec = (timeout - (tv.tv_sec*1000)) * 1000;
    int __r = 1;
    if(timeout != 0xFFFFFFFF) {
        __r = select(fd + 1, &set, NULL, NULL, &tv );
    }
    if(__r > 0) {
        static uint8_t readdummy[PIPE_BUF];
        result = 1;
        read(fd, readdummy, PIPE_BUF);
    } else if(__r < 0) {
        result = 0;
    } else {
        result = 0;
    }
    return result;
    }

    uint64_t CreateTimestampUs() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (((uint64_t)ts.tv_sec) * 1000*1000) + (ts.tv_nsec/1000);
    }
#endif


