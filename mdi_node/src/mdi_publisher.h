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

