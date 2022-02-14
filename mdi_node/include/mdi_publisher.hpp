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

#ifndef MDI_PUBLISHER_HPP_
#define MDI_PUBLISHER_HPP_

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
#include <memory>
#include <string>
#include <vector>
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

class MdiReceiveNode : public rclcpp::Node
{
public:
#ifdef AS_NODELET
  COMPOSITION_PUBLIC
  explicit MdiReceiveNode(const rclcpp::NodeOptions & option)
  : Node(MDI_NODE_NAME, option), m_pRxAPI(BplMeas_InvokeApi()->GetRxAPI())
  {
    MdiReceiveNode_Initializer();
  }
#else
  MdiReceiveNode()
  : Node(MDI_NODE_NAME), m_pRxAPI(BplMeas_InvokeApi()->GetRxAPI())
  {
    MdiReceiveNode_Initializer();
  }
#endif

  virtual ~MdiReceiveNode()
  {
    m_worker_thread_running = false;
    m_worker_thread->join();
    delete (m_worker_thread);
  }

protected:
  void evaluate_frame(
    std::unique_ptr<mdi_msgs::msg::Mdirawframe> pcache,
    const std::string & src_ip,
    rclcpp::Time * pSimTime);

  void mdi_reception_worker();

  uint32_t raw_convert_aveto_to_ros_msg(
    const std::string & src_ip,
    struct AvetoHeaderV2x1_Proto const * const pAveto,
    mdi_msgs::msg::MdiAvetoProfile & mdi_info,
    std_msgs::msg::Header & header);

private:
  MdiRx_Reception_interface_t const * const m_pRxAPI;
  std::thread * m_worker_thread;
  bool m_worker_thread_running;
  rclcpp::Publisher<mdi_msgs::msg::Mdirxapistatus>::SharedPtr m_api_status_publisher;
  rclcpp::Publisher<mdi_msgs::msg::Mdirawframe>::SharedPtr m_mdi_raw_publisher;
  rclcpp::Publisher<mdi_msgs::msg::MdiStatusFrame>::SharedPtr m_mdi_status_publisher;
  rclcpp::Publisher<mdi_msgs::msg::MdiCsi2Frame>::SharedPtr m_mdi_csi2_publisher;

  void MdiReceiveNode_Initializer();

#ifdef PERFORMANCE_MEAS
  bool load_file(const std::string & filename, std::vector<uint8_t> & data);

  void timer_callback();

  rclcpp::TimerBase::SharedPtr m_timed_dump_player;
#endif
};
#endif  // MDI_PUBLISHER_HPP_
