#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <stdlib.h>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mdi_node/msg/mdirxapistatus.hpp"
#include "mdi_node/msg/aveto_frame.hpp"
#include "mdi_node/msg/aveto_timebase.hpp"
#include "mdi_node/msg/mdirawframe.hpp"
#include "mdi_node/msg/mdi_csi2_frame.hpp"
#include "mdi_node/msg/mdi_status_frame.hpp"
#include "MDIRxAPI.h"
#include "MDI_DAQProt_Profile.h"

using namespace std::chrono_literals;
#define MDI_NODE_NAME "mdi_receiver"

/* sadly, neither ROS2, not iceoryx, support variable sized messages at the moment. this makes usage of
   zero-copy a little bit... impossible, at least if your data can be at any size */


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

void dfsa(mdi_node::msg::Mdirawframe* pcache) {

}

MDIRXAPI_API MdiRx_API_interface_t const*const (MDIRXAPI_DECL *BplMeas_DynInvokeApi)( void )=nullptr;
class MdiBasePublisher : public rclcpp::Node
{
  public:
    MdiBasePublisher()
    : Node(MDI_NODE_NAME), pRxAPI(BplMeas_DynInvokeApi()?BplMeas_DynInvokeApi()->GetRxAPI():nullptr)
    {
      if(!pRxAPI) throw std::runtime_error("mdi rx api was not properly laoded");
      uint32_t v = pRxAPI->GetApiVersion();
      RCLCPP_INFO(this->get_logger(), "MDI API Version: %d.%d.%d (%s)", (v>>16) & 0xFF, (v>>8)&0xFF, v&0xFF, (v&0x8000000)?"DEBUG":"Release");
      RCLCPP_INFO(this->get_logger(), "MDI RX ABI Version: %d", pRxAPI->info.version );
      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      
    
      
      worker_thread_running=true;
      worker_thread=new std::thread(&MdiBasePublisher::mdi_reception_worker, this);
       
    }

    virtual ~MdiBasePublisher() {
      worker_thread_running=false;
      worker_thread->join();
    }

  protected:
    void mdi_reception_worker() {
      uint64_t received_bytes=0;
      rclcpp::Publisher<mdi_node::msg::Mdirxapistatus>::SharedPtr api_status_publisher;
      rclcpp::Publisher<mdi_node::msg::Mdirawframe>::SharedPtr mdi_raw_publisher;
      rclcpp::Publisher<mdi_node::msg::MdiStatusFrame>::SharedPtr mdi_status_publisher;
      rclcpp::Publisher<mdi_node::msg::MdiCsi2Frame>::SharedPtr mdi_csi2_publisher;
      BplMeasFrameInfo_t FrameCache[256];
      
      uint32_t dwFrameCount = 256;

      pRxAPI->InitEx(BPLMEAS_LISTEN_ALL_IFC_ADDR, BPLMEAS_DEFAULT_RX_BASE_PORT, BPLMEAS_DEFAULT_RX_PORT_COUNT+1);
      pRxAPI->DeliverCorruptFrames(false);

      /* huge name for just a preallocation */
      pRxAPI->RegisterMemManager(
        [](size_t size_to_alloc, void* pMemMgr, void** pInstanceTag) -> void* {
          pMemMgr=pMemMgr;
          /* we're not entirely sure what we will get - so start with a raw message and swap it afterwards */
          mdi_node::msg::Mdirawframe* pcache=new mdi_node::msg::Mdirawframe(rosidl_runtime_cpp::MessageInitialization::SKIP);
          pcache->data.resize(size_to_alloc);
          *pInstanceTag=(void*)pcache;
          return pcache->data.data();
        },
        [](void* p, size_t size_to_alloc, void* pMemMgr, void** pInstanceTag) -> void* {
          p=p;
          pMemMgr=pMemMgr;
          mdi_node::msg::Mdirawframe* pcache=(mdi_node::msg::Mdirawframe*)*pInstanceTag;
          pcache->data.resize(size_to_alloc);
          return pcache->data.data();
        },
        [](void* p, void* pMemMgr, void** pInstanceTag) {
          p=p;
          pMemMgr=pMemMgr;
          pInstanceTag=pInstanceTag;
          /* yeah, we do actually nothing here. the message is used outside (in the publisher), so
             we do not cleanup anything. the publisher will handle this for sure */
        },
        this
      );

      api_status_publisher = this->create_publisher<mdi_node::msg::Mdirxapistatus>("mdi/rxapi/status", 10);
      mdi_raw_publisher = this->create_publisher<mdi_node::msg::Mdirawframe>("mdi/raw_daq", 512);
      mdi_csi2_publisher = this->create_publisher<mdi_node::msg::MdiCsi2Frame>("mdi/csi2_frame", 512);
      mdi_status_publisher = this->create_publisher<mdi_node::msg::MdiStatusFrame>("mdi/status", 32);

      WaitHandle_t hEvt = pRxAPI->GetDataEventHandle();
      pRxAPI->Start();
      RCLCPP_INFO(this->get_logger(), "reception started");

      uint32_t cnt=16;
      
      uint64_t TS = CreateTimestampUs();
      while(worker_thread_running /*&& rclcpp::ok()*/) {

        if (__eventWait(hEvt, 10)) {
          dwFrameCount = sizeof(FrameCache) / sizeof(BplMeasFrameInfo_t);
          if (pRxAPI->GetData(FrameCache, &dwFrameCount) == NOERROR) {
            for (uint32_t i = 0; i < dwFrameCount; i++) {
              received_bytes+=FrameCache[i].Size;

              std::unique_ptr<mdi_node::msg::Mdirawframe> pcache = std::unique_ptr<mdi_node::msg::Mdirawframe>((mdi_node::msg::Mdirawframe*)FrameCache[i].pInstanceTag);

              uint32_t used_size=FrameCache[i].Size;
              std::string src_ip=std::to_string((FrameCache[i].SrcIp)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>8)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>16)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>24)&0xFF);
              pRxAPI->FreeData(&FrameCache[i]);

              if(used_size < pcache->data.size()) {
                pcache->data.resize(used_size);
              }
              struct AvetoHeaderV2x1_Proto const*const pAveto=(struct AvetoHeaderV2x1_Proto const*const)pcache->data.data();
              struct SUniqueID_t const*const pUID = (struct SUniqueID_t const*const)&pAveto->frame.uiStreamID;

              switch(pUID->DataType) {
                case DAQPROT_PACKET_TYPE_CSI2_RAW_AGGREGATION: {
                  /* for CSI2 frames, we just gather the header information and swap the payload - no need to do any
                     expensive copies here */
                  auto csi2_msg=mdi_node::msg::MdiCsi2Frame(rosidl_runtime_cpp::MessageInitialization::SKIP);
                  uint32_t offset=raw_convert_aveto_to_ros(src_ip, pAveto, csi2_msg.mdi_info, csi2_msg.header);
                  csi2_msg.data.swap(pcache->data);
                  csi2_msg.offset_to_payload=offset;
                  mdi_csi2_publisher->publish(csi2_msg);
                } break;
                case DAQPROT_PACKET_TYPE_JSON_STATUS: {
                  /* the MDI device regularily sends some status information, as a large JSON string. This needs
                     to be copied from the payload (thanks, STL). But it's only every now and then, so it's ok. */
                  auto json_msg=mdi_node::msg::MdiStatusFrame(rosidl_runtime_cpp::MessageInitialization::SKIP);
                  uint32_t offset=raw_convert_aveto_to_ros(src_ip, pAveto, json_msg.mdi_info, json_msg.header);
                  json_msg.stati=std::string(pcache->data.begin()+offset, pcache->data.end());
                  mdi_status_publisher->publish(json_msg);
                } break;
                default: {
                  /* at this point it's something different and we need to implement it either here - or directly where it's needed */
                  raw_convert_aveto_to_ros(src_ip, pAveto, pcache->mdi_info, pcache->header);
                  mdi_raw_publisher->publish(std::move(pcache));
                } break;
              }


              /* now check what we have here actually and react accordingly */
              
            }
          }
        }

        uint64_t nTS = CreateTimestampUs();
        if( (nTS - TS) > 1000000) {
          BplMeasMdiReceptionStatistics_t statistics;
          if (pRxAPI->GetStatistics(&statistics) == NOERROR) {
            auto message = mdi_node::msg::Mdirxapistatus();
            message.received_good_daq_frames+=statistics.NewCompletedFrames;
            message.timeout_daq_frames+=statistics.NewTimeoutFrames;
            message.tp_corrupt_daq_frames+=statistics.NewCorruptFrames+statistics.NewDiscardedFrames;
            message.received_bytes+=received_bytes;
            message.received_bandwidth_mib=received_bytes/1048576.f;
            api_status_publisher->publish(message);
            received_bytes=0;
          }
          TS = nTS;
        }


        std::this_thread::sleep_for(500ms);
      }
      pRxAPI->Stop();
      pRxAPI->Deinit();
      RCLCPP_INFO(this->get_logger(), "reception stopped");
    }

    uint32_t raw_convert_aveto_to_ros(std::string& src_ip, struct AvetoHeaderV2x1_Proto const*const pAveto, mdi_node::msg::MdiAvetoProfile& mdi_info, std_msgs::msg::Header& header  ) {
      struct SUniqueID_t const*const pUID = (struct SUniqueID_t const*const)&pAveto->frame.uiStreamID;
      uint64_t used_timestamp=0;

      mdi_info.src_ip=src_ip;
      mdi_info.payload_offset=pAveto->frame.uiPayloadOffs;
      mdi_info.frame_info.stream_id=pAveto->frame.uiStreamID;
      mdi_info.frame_info.device_instance=pUID->Instance;
      mdi_info.frame_info.port_number=pUID->Channel;
      mdi_info.frame_info.port_sub_index=pUID->Index;
      mdi_info.frame_info.cycle_counter=pAveto->cycle.uiCycleCount;
      mdi_info.frame_info.data_type=pUID->DataType;

      /* The timesynchronization is a very extensive feature, therefore we to the import to ROS2 explitly - so we
         have a chance to add some comments. */

      /* MDI Profile of Aveto always uses time[0] as a MDI-local monotonic nanosecond counter as reference 
         for any other time domain - but, by design, this counter cannot and will not carry the 
         synchronized flag. For extensive analysis this counter can be used to detect, evaluate and correct
         any time jumps/gaps occring in the environment (temporary lost of GPS fix on grandmaster, etc...). */
      mdi_info.time[0].timebase_type=pAveto->time[0].uiTimebaseType;  // TB_TYPE_LOCAL_NS
      mdi_info.time[0].timebase_flag=pAveto->time[0].uiTimebaseFlags; // TB_FLAG_UNSYNCED
      mdi_info.time[0].timestamp=pAveto->time[0].uiTimestamp;

      /* time[1] of MDI Profile will contain the TAI time (or rather 802.1AS Domain 0, to be correct), except if
         the MDI device is explictly configured to use UTC - then this time will be in UTC. For most applications
         UTC is discouraged because of the leap seconds, as this adds additional effort to the later evaluation and
         is generally error prone. One of the most typical effects is a GPS receiver getting the first fix of
         a power cycle - you will always get a time jump of several seconds in this case. However some environments
         still rely on UTC instead of TAI. */
      mdi_info.time[1].timebase_type=pAveto->time[1].uiTimebaseType;  // TB_TYPE_TAI or TB_TYPE_UTC
      mdi_info.time[1].timebase_flag=pAveto->time[1].uiTimebaseFlags; // TB_FLAG_UNSYNCED
      mdi_info.time[1].timestamp=pAveto->time[1].uiTimestamp;
      used_timestamp=pAveto->time[1].uiTimestamp; /* as ROS2 uses TAI out of the box, we just assume noone uses UTC */
      
      {
        static bool there_was_no_utc=true;
        if(pAveto->time[1].uiTimebaseType==DAQPROT_TIMEBASE_UTC && there_was_no_utc) {
          RCLCPP_ERROR(this->get_logger(), "at least MDI device at %s is using UTC instead of TAI - check your configuration", src_ip.c_str() );
          there_was_no_utc=false;
        }
      }

      /* time[2] of the MDI Profile always contains a WCD (Working Clock Domain) timestamp - if supplied.
         WCD is typically transported via 802.1AS Domain 1 and is defined as synchronized monotonic increasing
         timebase in contrast to TAI as timebase. */
      mdi_info.time[2].timebase_type=pAveto->time[2].uiTimebaseType;  // TB_TYPE_WCD
      mdi_info.time[2].timebase_flag=pAveto->time[2].uiTimebaseFlags; // TB_FLAG_UNSYNCED
      mdi_info.time[2].timestamp=pAveto->time[2].uiTimestamp;

      header.frame_id="MDILink"+std::to_string(pUID->Instance);
      header.stamp.sec=used_timestamp / 1000000000ULL;
      header.stamp.nanosec=used_timestamp - (header.stamp.sec * 1000000000ULL);

      return pAveto->frame.uiPayloadOffs;
    }

  private:
   
    MdiRx_Reception_interface_t const*const pRxAPI;
    std::thread* worker_thread;
    bool worker_thread_running;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MdiBasePublisher>());
  rclcpp::shutdown();
  return 0;
}


#ifdef WIN32
  /* to do as soon as ros2-win setup is setup */
#else
void* libmdirxapi_handle=nullptr;
__attribute__((constructor))
static void mdi_receiver_impl_load() {
    /* be very careful with this function: the initialization order of global/static variables with respect to this function depends on the moon phase or so */
    std::string location="";
    Dl_info dl_info;
    dladdr((void *)mdi_receiver_impl_load, &dl_info);
    size_t src_s=strlen(dl_info.dli_fname);
    location.resize(src_s);
    memcpy((void*)location.c_str(), dl_info.dli_fname, src_s);
    /* we need to fiddle a little bit here to get the path to us */
    location=location.substr(0, location.rfind("/"))+"/libmdirxapi.so";
    libmdirxapi_handle=dlopen(location.c_str(), RTLD_NOW|RTLD_GLOBAL);
    if(!libmdirxapi_handle) {
      RCLCPP_ERROR(rclcpp::get_logger(MDI_NODE_NAME), "failed to load libmdirxapi at %s", location.c_str() );
    } else {
      RCLCPP_INFO(rclcpp::get_logger(MDI_NODE_NAME), "loaded libmdirxapi at %p", libmdirxapi_handle);
      *(void**)&BplMeas_DynInvokeApi = dlsym(libmdirxapi_handle, "BplMeas_InvokeApi");
      RCLCPP_DEBUG(rclcpp::get_logger(MDI_NODE_NAME), "BplMeas_DynInvokeApi at %p", (void*)BplMeas_DynInvokeApi);
    }
}

__attribute__((destructor)) 
static void mdi_receiver_impl_unload() {
  if(libmdirxapi_handle) {
    dlclose(libmdirxapi_handle);
  }
}
#endif