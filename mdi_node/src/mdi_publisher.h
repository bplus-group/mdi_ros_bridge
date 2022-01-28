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

#pragma once

#ifdef AS_NODELET
#include "visibility_control.h"
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mdi_msgs/msg/mdirxapistatus.hpp>
#include <mdi_msgs/msg/aveto_frame.hpp>
#include <mdi_msgs/msg/aveto_timebase.hpp>
#include <mdi_msgs/msg/mdirawframe.hpp>
#include <mdi_msgs/msg/mdi_csi2_frame.hpp>
#include <mdi_msgs/msg/mdi_status_frame.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include "MDIRxAPI.h"
#pragma GCC diagnostic pop
#include "MDI_DAQProt_Profile.h"
#define MDI_NODE_NAME "mdi_receiver"

#ifdef WIN32
int __eventWait(HANDLE h, uint32_t timeout);
#else
int __eventWait(int fd, uint32_t timeout);
#endif
uint64_t CreateTimestampUs();
MdiRx_API_interface_t const* BplMeas_DynInvokeApi(void);

