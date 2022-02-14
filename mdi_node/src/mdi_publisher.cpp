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

#include "mdi_publisher.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

/* currently neither ROS2, nor iceoryx, support variable sized messages at the moment. this makes usage of
   zero-copy a little bit... impossible, at least if your data can be at any size */


void MdiReceiveNode::evaluate_frame(std::unique_ptr<mdi_msgs::msg::Mdirawframe> pcache,
                                    const std::string& src_ip,
                                    rclcpp::Time* pSimTime = nullptr) {
    const struct AvetoHeaderV2x1_Proto* pAveto = (struct AvetoHeaderV2x1_Proto const*)pcache->data.data();
    const struct SUniqueID_t* pUID = (struct SUniqueID_t const*)&pAveto->frame.uiStreamID;

    switch(pUID->DataType) {
      case DAQPROT_PACKET_TYPE_CSI2_RAW_AGGREGATION: {
        /* for CSI2 frames, we just gather the header information and swap the payload - no need to do any
            expensive copies here */
        const struct AvetoHeaderV2x1_CSI2Raw* pCsi2 = (struct AvetoHeaderV2x1_CSI2Raw const*)pAveto;
        auto csi2_msg = mdi_msgs::msg::MdiCsi2Frame(rosidl_runtime_cpp::MessageInitialization::SKIP);
        uint32_t offset = raw_convert_aveto_to_ros_msg(src_ip, pAveto, csi2_msg.mdi_info, csi2_msg.header);
        csi2_msg.data.swap(pcache->data);
        csi2_msg.offset_to_payload = offset;
        csi2_msg.number_lines = pCsi2->CSI2RawLines.uiLineCount;
        if(pSimTime) {
          csi2_msg.header.stamp = *pSimTime;
        }
        m_mdi_csi2_publisher->publish(csi2_msg);
      } break;
      case DAQPROT_PACKET_TYPE_JSON_STATUS: {
        /* the MDI device regularily sends some status information, as a large JSON string. This needs
            to be copied from the payload (thanks, STL). But it's only every now and then, so it's ok. */
        auto json_msg = mdi_msgs::msg::MdiStatusFrame(rosidl_runtime_cpp::MessageInitialization::SKIP);
        uint32_t offset = raw_convert_aveto_to_ros_msg(src_ip, pAveto, json_msg.mdi_info, json_msg.header);
        json_msg.stati = std::string(pcache->data.begin()+offset, pcache->data.end());
        m_mdi_status_publisher->publish(json_msg);
      } break;
      default: {
        /* at this point it's something different and we need to implement it either here - or directly
         * where it's needed */
        raw_convert_aveto_to_ros_msg(src_ip, pAveto, pcache->mdi_info, pcache->header);
        m_mdi_raw_publisher->publish(std::move(pcache));
      } break;
    }
}
void MdiReceiveNode::mdi_reception_worker() {
  uint64_t received_bytes = 0;

  BplMeasFrameInfo_t FrameCache[256];

  uint32_t dwFrameCount = 256;

  m_pRxAPI->InitEx(BPLMEAS_LISTEN_ALL_IFC_ADDR,
                   BPLMEAS_DEFAULT_RX_BASE_PORT,
                   BPLMEAS_DEFAULT_RX_PORT_COUNT+1);
  m_pRxAPI->DeliverCorruptFrames(false);

  /* huge name for just a preallocation */
  m_pRxAPI->RegisterMemManager(
    [](size_t size_to_alloc, void* pMemMgr, void** pInstanceTag) -> void* {
      (void) pMemMgr;
      /* we're not entirely sure what we will get - so start with a raw message and swap it afterwards */
      mdi_msgs::msg::Mdirawframe* pcache = new mdi_msgs::msg::Mdirawframe(
               rosidl_runtime_cpp::MessageInitialization::SKIP);
      pcache->data.resize(size_to_alloc);
      *pInstanceTag =  reinterpret_cast<void*>(pcache);
      return pcache->data.data();
    },
    [](void* p, size_t size_to_alloc, void* pMemMgr, void** pInstanceTag) -> void* {
      (void) p;
      (void) pMemMgr;
      mdi_msgs::msg::Mdirawframe* pcache = (mdi_msgs::msg::Mdirawframe*)*pInstanceTag;
      pcache->data.resize(size_to_alloc);
      return pcache->data.data();
    },
    [](void* p, void* pMemMgr, void** pInstanceTag) {
      (void) p;
      (void) pMemMgr;
      (void) pInstanceTag;
      /* yeah, we do actually nothing here. the message is used outside (in the publisher), so
         we do not cleanup anything. the publisher will handle this for sure */
    },
    this
  );

  WaitHandle_t hEvt = m_pRxAPI->GetDataEventHandle();
  m_pRxAPI->Start();
  RCLCPP_INFO(this->get_logger(), "reception started");

  uint64_t TS = CreateTimestampUs();
  while(m_worker_thread_running /*&& rclcpp::ok()*/) {
    if (__eventWait(hEvt, 10)) {
      dwFrameCount = sizeof(FrameCache) / sizeof(BplMeasFrameInfo_t);
      if (m_pRxAPI->GetData(FrameCache, &dwFrameCount) == NOERROR) {
        for (uint32_t i = 0; i < dwFrameCount; i++) {
          received_bytes+=FrameCache[i].Size;
          std::unique_ptr<mdi_msgs::msg::Mdirawframe> pcache =
              std::unique_ptr<mdi_msgs::msg::Mdirawframe>(
                      (mdi_msgs::msg::Mdirawframe*)FrameCache[i].pInstanceTag
              );

          uint32_t used_size = FrameCache[i].Size;
          std::string src_ip = std::to_string((FrameCache[i].SrcIp)&0xFF) + "." +
                               std::to_string((FrameCache[i].SrcIp>>8)&0xFF) + "." +
                               std::to_string((FrameCache[i].SrcIp>>16)&0xFF) + "." +
                               std::to_string((FrameCache[i].SrcIp>>24)&0xFF);
          m_pRxAPI->FreeData(&FrameCache[i]);

          if (used_size < pcache->data.size()) {
            pcache->data.resize(used_size);
          }
          evaluate_frame(std::move(pcache), src_ip);
        }
      }
    } else {
      std::this_thread::sleep_for(10ms);
    }

    uint64_t nTS = CreateTimestampUs();
    if( (nTS - TS) > 1000000) {
      BplMeasMdiReceptionStatistics_t statistics;
      if (m_pRxAPI->GetStatistics(&statistics) == NOERROR) {
        auto message = mdi_msgs::msg::Mdirxapistatus();
        message.received_good_daq_frames += statistics.NewCompletedFrames;
        message.timeout_daq_frames += statistics.NewTimeoutFrames;
        message.tp_corrupt_daq_frames += statistics.NewCorruptFrames+statistics.NewDiscardedFrames;
        message.received_bytes += received_bytes;
        message.received_bandwidth_mib = received_bytes/1048576.f;
        m_api_status_publisher->publish(message);
        received_bytes = 0;
      }
      TS = nTS;
    }
  }
  m_pRxAPI->Stop();
  m_pRxAPI->Deinit();
  RCLCPP_INFO(this->get_logger(), "reception stopped");
}

uint32_t MdiReceiveNode::raw_convert_aveto_to_ros_msg(const std::string& src_ip,
                                                      struct AvetoHeaderV2x1_Proto const*const pAveto,
                                                      mdi_msgs::msg::MdiAvetoProfile& mdi_info,
                                                      std_msgs::msg::Header& header) {
  const struct SUniqueID_t* pUID = (struct SUniqueID_t const*)&pAveto->frame.uiStreamID;
  uint64_t used_timestamp = 0;

  mdi_info.src_ip = src_ip;
  mdi_info.payload_offset = pAveto->frame.uiPayloadOffs;
  mdi_info.frame_info.stream_id = pAveto->frame.uiStreamID;
  mdi_info.frame_info.device_instance = pUID->Instance;
  mdi_info.frame_info.port_number = pUID->Channel;
  mdi_info.frame_info.port_sub_index = pUID->Index;
  mdi_info.frame_info.cycle_counter = pAveto->cycle.uiCycleCount;
  mdi_info.frame_info.data_type = pUID->DataType;

  /* The timesynchronization is a very extensive and complex feature of MDIs, where ROS2 supports only a small
     subset. For now, we'll just assume ROS2 uses TAI and is synchonizing the MDI from the same source. */

  /* MDI Profile of Aveto always uses time[0] as a MDI-local monotonic nanosecond counter as reference 
     for any other time domain - but, by design, this counter cannot and will not carry the 
     synchronized flag. For extensive analysis this counter can be used to detect, evaluate and correct
     any time jumps/gaps occring in the environment (temporary lost of GPS fix on grandmaster, etc...). */
  mdi_info.time[0].timebase_type = pAveto->time[0].uiTimebaseType;   // TB_TYPE_LOCAL_NS
  mdi_info.time[0].timebase_flag = pAveto->time[0].uiTimebaseFlags;  // TB_FLAG_UNSYNCED
  mdi_info.time[0].timestamp = pAveto->time[0].uiTimestamp;

  /* time[1] of MDI Profile will contain the TAI time (or rather 802.1AS Domain 0, to be correct), except if
     the MDI device is explictly configured to use UTC - then this time will be in UTC. For most applications
     UTC is discouraged because of the leap seconds, as this adds additional effort to the later evaluation and
     is generally error prone. One of the most typical effects is a GPS receiver getting the first fix of
     a power cycle - you will always get a time jump of several seconds in this case. However some environments
     still rely on UTC instead of TAI. */
  mdi_info.time[1].timebase_type = pAveto->time[1].uiTimebaseType;   // TB_TYPE_TAI or TB_TYPE_UTC
  mdi_info.time[1].timebase_flag = pAveto->time[1].uiTimebaseFlags;  // TB_FLAG_UNSYNCED
  mdi_info.time[1].timestamp = pAveto->time[1].uiTimestamp;
  /* as ROS2 uses TAI out of the box, we just assume noone uses UTC */
  used_timestamp = pAveto->time[1].uiTimestamp;

  {
    static bool there_was_no_utc = true;
    if(pAveto->time[1].uiTimebaseType == DAQPROT_TIMEBASE_UTC && there_was_no_utc) {
      RCLCPP_ERROR(this->get_logger(), "at least MDI device at %s is using UTC instead of TAI"
                                       " - check your configuration", src_ip.c_str() );
      there_was_no_utc = false;
    }
  }

  /* time[2] of the MDI Profile always contains a WCD (Working Clock Domain) timestamp - if supplied.
     WCD is typically transported via 802.1AS Domain 1 and is defined as synchronized monotonic increasing
     timebase in contrast to TAI as timebase. */
  mdi_info.time[2].timebase_type = pAveto->time[2].uiTimebaseType;   // TB_TYPE_WCD
  mdi_info.time[2].timebase_flag = pAveto->time[2].uiTimebaseFlags;  // TB_FLAG_UNSYNCED
  mdi_info.time[2].timestamp = pAveto->time[2].uiTimestamp;

  header.frame_id = "MDILink"+std::to_string(pUID->Instance);
  header.stamp.sec = used_timestamp / 1000000000ULL;
  header.stamp.nanosec = used_timestamp - (header.stamp.sec * 1000000000ULL);

  return pAveto->frame.uiPayloadOffs;
}

void MdiReceiveNode::MdiReceiveNode_Initializer() {
  if(!m_pRxAPI) throw std::runtime_error("mdi rx api was not properly laoded");
  uint32_t v = m_pRxAPI->GetApiVersion();
  RCLCPP_INFO(this->get_logger(), "MDI API Version: %d.%d.%d (%s)",
          (v>>16) & 0xFF, (v>>8)&0xFF, v&0xFF, (v&0x8000000)?"DEBUG":"Release");
  RCLCPP_INFO(this->get_logger(), "MDI RX ABI Version: %d", m_pRxAPI->info.version );

  m_api_status_publisher = this->create_publisher<mdi_msgs::msg::Mdirxapistatus>("mdi/rxapi/status", 10);
  m_mdi_raw_publisher = this->create_publisher<mdi_msgs::msg::Mdirawframe>("mdi/raw_daq", 512);
  m_mdi_csi2_publisher = this->create_publisher<mdi_msgs::msg::MdiCsi2Frame>("mdi/csi2_frame", 512);
  m_mdi_status_publisher = this->create_publisher<mdi_msgs::msg::MdiStatusFrame>("mdi/status", 32);

  m_worker_thread_running = true;
  m_worker_thread = new std::thread(&MdiReceiveNode::mdi_reception_worker, this);

  #ifdef PERFORMANCE_MEAS
  this->m_timed_dump_player = this->create_wall_timer(500ms,
                                        std::bind(&MdiReceiveNode::timer_callback, this));
  #endif
}

#ifdef PERFORMANCE_MEAS
bool MdiReceiveNode::load_file(const std::string& filename, std::vector<uint8_t>& data) {
  FILE* pDump = fopen(filename.c_str(), "rb");
  if(pDump) {
    fseek(pDump, 0, SEEK_END);
    size_t size = ftell(pDump);
    data.resize(size);
    fseek(pDump, 0, SEEK_SET);
    fread(data.data(), 1, data.size(), pDump);
    fclose(pDump);
  }
  return pDump;
}

void MdiReceiveNode::timer_callback()
{
    std::string src_ip = "127.0.0.1";
    mdi_msgs::msg::Mdirawframe* pcache =
        new mdi_msgs::msg::Mdirawframe(rosidl_runtime_cpp::MessageInitialization::SKIP);
    std::unique_ptr<mdi_msgs::msg::Mdirawframe> rosframe =
        std::unique_ptr<mdi_msgs::msg::Mdirawframe>(pcache);
    rclcpp::Time mytime = rclcpp::Node::now();
    std::vector<uint8_t> data;
    if(load_file("/tmp/YUV422-8.dump", data )) {
        rosframe->data.swap(data);
        evaluate_frame(std::move(rosframe), src_ip, &mytime);
    }
}
#endif

#ifndef AS_NODELET
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MdiReceiveNode>());
  rclcpp::shutdown();
  return 0;
}
#else

#include <rclcpp_components/register_node_macro.hpp>
// Register the component with class_loader
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MdiReceiveNode)
#endif



