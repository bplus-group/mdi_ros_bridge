#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mdi_node/msg/mdirxapistatus.hpp"
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

              mdi_node::msg::Mdirawframe* pcache=(mdi_node::msg::Mdirawframe*)FrameCache[i].pInstanceTag;


              uint32_t used_size=FrameCache[i].Size;
              std::string src_ip=std::to_string((FrameCache[i].SrcIp)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>8)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>16)&0xFF) + "." + 
                             std::to_string((FrameCache[i].SrcIp>>24)&0xFF);
              pRxAPI->FreeData(&FrameCache[i]);

              /* now check what we have here actually and react accordingly */
              publish_data(src_ip, used_size, std::unique_ptr<mdi_node::msg::Mdirawframe>(pcache));



              //mdi_raw_publisher->publish();
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

    void extract_meta_data(mdi_node::msg::MdiAvetoProfile& profile, struct AvetoHeaderV2x1_Proto const*const pAveto, std::string& src_ip, uint16_t data_type) {
      struct SUniqueID_t const*const pUID = (struct SUniqueID_t const*const)&pAveto->frame.uiStreamID;

      profile.src_ip=src_ip;
      profile.payload_offset=pAveto->frame.uiPayloadOffs;
      profile.frame_info.stream_id=pAveto->frame.uiStreamID;
      profile.frame_info.device_instance=pUID->Instance;
      profile.frame_info.port_number=pUID->Channel;
      profile.frame_info.port_sub_index=pUID->Index;
      profile.frame_info.data_type=data_type;


      profile.frame_info.cycle_counter=pAveto->cycle.uiCycleCount;




    }

    void publish_data(std::string& src_ip, uint32_t used_size, std::unique_ptr<mdi_node::msg::Mdirawframe> raw_message ) {

      if(used_size < raw_message->data.size()) {
        raw_message->data.resize(used_size);
      }
      struct AvetoHeaderV2x1_Proto const*const pAveto=(struct AvetoHeaderV2x1_Proto const*const)raw_message->data.data();
      struct SUniqueID_t const*const pUID = (struct SUniqueID_t const*const)&pAveto->frame.uiStreamID;

      switch(pUID->DataType) {
        case DAQPROT_PACKET_TYPE_CSI2_RAW_AGGREGATION: {
          auto msg=mdi_node::msg::MdiCsi2Frame(rosidl_runtime_cpp::MessageInitialization::SKIP);
          extract_meta_data(msg.mdi_info, pAveto, src_ip, pUID->DataType);
          msg.data.swap(raw_message.data);
        } break;
        case DAQPROT_PACKET_TYPE_JSON_STATUS: {
          auto msg=mdi_node::msg::MdiCsi2Frame(rosidl_runtime_cpp::MessageInitialization::SKIP);
          extract_meta_data(msg.mdi_info, pAveto, src_ip, pUID->DataType);
          msg.data.swap(raw_message.data);
        } break;
        default: {
          extract_meta_data(raw_message->mdi_info, pAveto, src_ip, 0);
        } break;
      }


    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! ";
    }

    rclcpp::TimerBase::SharedPtr timer_;
    
    
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