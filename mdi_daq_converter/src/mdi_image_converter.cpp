#include "visibility_control.h"
#include <memory>
#include <tuple>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <mdi_msgs/msg/aveto_frame.hpp>
#include <mdi_msgs/msg/aveto_timebase.hpp>
#include <mdi_msgs/msg/mdi_csi2_frame.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "MDI_DAQProt_Profile.h"
using std::placeholders::_1;

/*
  MDI
   |------ Instance 0
   |          |---------- Channel 0 (ChannelMapping)
   |          |               |--------- VC 0 (VCMapping)
   |          |               |           |--------- Data Type (DataTypeMapping)

*/

struct frame_content {
  frame_content() {
    lines=0;
    length=0;
    init=false;
  }
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  uint32_t lines;
  uint16_t length;
  bool init;
};
typedef std::unordered_map<uint8_t, frame_content> frame_content_type;

struct port_context {
  port_context() {
    init=false;
  }
  bool init;
  std::array<frame_content_type,4> frame_content;
};

typedef std::unordered_map<uint16_t, port_context> InstanceChannelMapping;

class MdiConverterNode : public rclcpp::Node {
  public:
    MdiConverterNode(const rclcpp::NodeOptions &option)
    : Node("mdi_converter", option)
    { 
      subscription_ = this->create_subscription<mdi_msgs::msg::MdiCsi2Frame>("mdi/csi2_frame", 64, std::bind(&MdiConverterNode::topic_callback, this, _1));
    }

    virtual ~MdiConverterNode() { }



  private:
    
    bool parse_frame(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr& msg, port_context& context ) {
      uint32_t line_count=0;
      const SCSI2RawLineHeader_t* pLine=(const SCSI2RawLineHeader_t*)&msg->data[msg->offset_to_payload];
      
      while( (void*)pLine < (void*)&msg->data[msg->data.size()] ) {
        auto act_content=&context.frame_content[pLine->sCSI2Header.sShortHeader.uiVirtualChannel][pLine->sCSI2Header.sShortHeader.uiDataType];
        line_count++;
        if(line_count > msg->number_lines) {
          RCLCPP_ERROR(this->get_logger(), "more CSI2 lines found in frame than stated in header - seems to be something corrupt/wrong configured.");
          return false;
        }
        if(!act_content->init) {
          act_content->lines=1;
          if(pLine->sCSI2Header.sShortHeader.uiDataType < 0x10){
            /* short packages do not have any payload */
            act_content->length=0;
            pLine++;
          } else {
            /* long types have a payload and are a little bit more complex */
            act_content->length=pLine->sCSI2Header.sLongHeader.uiWordCount;
            const SCSI2RawLineFooter_t* pEnd = (const SCSI2RawLineFooter_t*)&((const uint8_t*)(pLine+1))[pLine->sCSI2Header.sLongHeader.uiWordCount];
            pLine = (const SCSI2RawLineHeader_t*)(pEnd+1);
            act_content->publisher=this->create_publisher<sensor_msgs::msg::Image>("mdi/" + 
                                                                                    std::to_string(msg->mdi_info.frame_info.device_instance) + "/" +
                                                                                    std::to_string(msg->mdi_info.frame_info.port_number) + "/" +
                                                                                    std::to_string(pLine->sCSI2Header.sShortHeader.uiVirtualChannel) + "/" +
                                                                                    std::to_string(pLine->sCSI2Header.sShortHeader.uiDataType)
                                                                                    , 32);
          }
          act_content->init=true;
        } else {
          act_content->lines++;
          if(pLine->sCSI2Header.sShortHeader.uiDataType < 0x10){
            /* short packages do not have any payload */
            pLine++;
          } else {
            if(act_content->length != pLine->sCSI2Header.sLongHeader.uiWordCount) {
              RCLCPP_ERROR(this->get_logger(), "CSI2 line lenght is not constant! Check if source is a image and if everythin is configured correctly");
              /* we'll use the new length, but abort processing (to not spam to much errors). if the new length is correct, the next frame will prove this,
                 else we'll just get another error */
              act_content->length = pLine->sCSI2Header.sLongHeader.uiWordCount;
              return false;
            }
            const SCSI2RawLineFooter_t* pEnd = (const SCSI2RawLineFooter_t*)&((const uint8_t*)(pLine+1))[pLine->sCSI2Header.sLongHeader.uiWordCount];
            pLine = (const SCSI2RawLineHeader_t*)(pEnd+1);
          }
        }
      }
      return true;
    }

    void topic_callback(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr msg) {
      uint8_t Instance=msg->mdi_info.frame_info.device_instance;
      uint8_t Port=msg->mdi_info.frame_info.port_number;
      uint16_t InstanceChannel=((Instance)<<8) | (Port);

      /* if we never saw this instance and port, we're checking what's in the frame. 
         we assume that every frame will be setup identically (i.e. same witdh/height of image),
         so we will cache this and only update if we find something different during converting.
      */
      if (!meta_cache[InstanceChannel].init) {  
        
        parse_frame(msg, meta_cache[InstanceChannel] );


        /* Topci: mdi/<instance>/<port>/<virtual channel>/<data_type> */
        /* Topic: mdi/0/1/3/YUV422-8 */
       // meta_cache[InstanceChannel][0][0x2C].publisher = this->create_publisher<sensor_msgs::msg::Image>("mdi/" + std::to_string(Instance) + "/" + std::to_string(Port) + "/0/" + "raw12");
      }
    }
    rclcpp::Subscription<mdi_msgs::msg::MdiCsi2Frame>::SharedPtr subscription_;
    InstanceChannelMapping meta_cache;
};

#include <rclcpp_components/register_node_macro.hpp>
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MdiConverterNode)