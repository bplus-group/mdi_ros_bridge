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

#include "visibility_control.h"
#include <memory>
#include <tuple>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <mdi_msgs/msg/aveto_frame.hpp>
#include <mdi_msgs/msg/aveto_timebase.hpp>
#include <mdi_msgs/msg/mdi_csi2_frame.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>

#include "MDI_DAQProt_Profile.h"
using std::placeholders::_1;

/*
  Our converter separates the individual CSI2 lines based on the sending MDI (instance) and the physical
  port of the MDI (channel), the virtual channel of the CSI2 data and the actual data type.

  virtually:
  MDI
   |------ Instance 0
   |          |---------- Channel 0 (ChannelMapping)
   |          |               |--------- VC 0 (VCMapping)
   |          |               |           |--------- Data Type (DataTypeMapping)


  actual: we combine instance and channel as "instance_channel_mapping" as the additional effort to manage those two 
  creates more writing overhead than it solves.

  instance_channel_mapping
          | ------------------- port_context (e.g. 0/0)
          |                           |-------------------- frame_content_type [0]
          |                           |                           |---------------- frame_content (0x1E)
          |                           |                           |                       | - image_publisher
          |                           |                           |                       | - camera_info
          |                           |                           |                       | - other image contextual data


  Note one parse_frame() and convert_frame():
  While those two methods are very similar in structure they serve a separate purpose. parse_frame() initializes and validates 
  the data types while convert_frames does the actual conversion. It would be too much of a hassle and complex to combine both.

  Note for future extensions:
  Depending on the setups, it might be a good idea to run parse_frame() by a thread per MDI instance to have a somewhat
  parallel processing of initializing the mapping-tree. Afterwards it would be safe to do the actual image conversion
  in separate threads or maybe use openmp for more complex conversions.
*/

struct frame_content {
  frame_content() {
    lines=0;
    length=0;
    init=false;
    image_pos=0;
    image_lines=0;
    length_odd=0;
  }
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info;
  uint32_t lines;
  uint16_t length;
  uint16_t length_odd;
  uint32_t width;
  bool init;
  sensor_msgs::msg::Image Image;
  sensor_msgs::msg::CameraInfo CamInfo;
  uint32_t image_pos;
  uint32_t image_lines;
};
typedef std::unordered_map<uint8_t, frame_content> frame_content_type;

struct port_context {
  port_context() {
    init=false;
  }
  bool init;
  std::array<frame_content_type,4> frame_content;
};

typedef std::unordered_map<uint16_t, port_context> instance_channel_mapping;

class MdiConverterNode : public rclcpp::Node {
  public:
    MdiConverterNode(const rclcpp::NodeOptions &option)
    : Node("mdi_converter", option)
    { 
      subscription_ = this->create_subscription<mdi_msgs::msg::MdiCsi2Frame>("mdi/csi2_frame", 64, std::bind(&MdiConverterNode::topic_callback, this, _1));
    }

    virtual ~MdiConverterNode() { }



  private:
    
    /* we have some CSI2 data types with interleaving line lengths which need more effort to validate a common line length */
    bool weird_line_behavior(uint8_t dt) {
      switch(dt&0x3F) {
        case 0x18: // YUV420-8
        case 0x1C: // YUV420-8C
        case 0x19: // YUV420-10
        case 0x1D: // YUV420-10C
        return true;
      }
      return false;
    }

    /* some CSI2 data types will map directly to a ROS2 type, others might need more "adjustment" */
    const char* csi2_type_to_ros2(uint8_t dt) {
      switch(dt&0x3F) {
        case 0x1E: // YUV422-8
          return sensor_msgs::image_encodings::YUV422;
        //case 0x2A: // RAW8
        //  return sensor_msgs::image_encodings::MONO8;
        //case 0x24: // RGB888
        //  return sensor_msgs::image_encodings::RGB8;
      }
      return "N/A";
    }

    /* depending on the data type, we need to derive the actual pixels from the number of bytes */
    uint32_t convert_line_length(uint8_t dt, uint32_t line, uint16_t num_bytes) {
      switch(dt&0x3F) {
        case 0x18: // YUV420-8
        case 0x1C: // YUV420-8C
          return (line&1)?num_bytes:num_bytes/2; // odd lines have 8bpp, even have 16bpp
        case 0x19: // YUV420-10
        case 0x1D: // YUV420-10C
          return (line&1)?(num_bytes*4)/5:(num_bytes*4)/10; // odd lines have 8bpp, even have 16bpp
        case 0x1A: // YUV420-8L
          return (num_bytes*2)/3;
        case 0x1E: // YUV422-8
          return num_bytes/2;
        case 0x1F: // YUV422-10
          return (num_bytes*2)/5;

        case 0x20: // RGB444
        case 0x21: // RGB555
        case 0x22: // RGB565
          return num_bytes/2;
        case 0x23: // RGB666
          return (num_bytes*4)/9;
        case 0x24: // RGB888
          return num_bytes/3;

        case 0x28: // RAW6
          return (num_bytes*4)/3;
        case 0x29: // RAW7
          return (num_bytes*8)/7;
        case 0x2A: // RAW8
          return num_bytes;
        case 0x2B: // RAW10
          return (num_bytes*4)/5;
        case 0x2C: // RAW12
          return (num_bytes*2)/3;
        case 0x2D: // RAW14
          return (num_bytes*4)/7;
        
        default:
        return num_bytes;
      }
    }

    bool parse_frame(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr& msg, port_context& context ) {
      uint32_t line_count=0;
      SCSI2RawLineCollection_t* pCSI2 = (SCSI2RawLineCollection_t*)&msg->data[msg->offset_to_payload];
      const SCSI2RawLineHeader_t* pLine=(const SCSI2RawLineHeader_t*)(pCSI2+1);

      /* iterate through all CSI2 lines until we reach the end of our memory block */
      while( (void*)pLine < (void*)&msg->data[msg->data.size()] ) {
        
        /* grab something shorter based on virtual channel and data type */
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
            act_content->width=convert_line_length(pLine->sCSI2Header.sShortHeader.uiDataType, act_content->lines ,act_content->length);
            const SCSI2RawLineFooter_t* pEnd = (const SCSI2RawLineFooter_t*)&((const uint8_t*)(pLine+1))[pLine->sCSI2Header.sLongHeader.uiWordCount];
            pLine = (const SCSI2RawLineHeader_t*)(pEnd+1);
            std::string pub_name="mdi/instance_" + 
                                  std::to_string(msg->mdi_info.frame_info.device_instance) + "/port_" +
                                  std::to_string(msg->mdi_info.frame_info.port_number) + "/vc_" +
                                  std::to_string(pLine->sCSI2Header.sShortHeader.uiVirtualChannel) + "/dt_" +
                                  std::to_string(pLine->sCSI2Header.sShortHeader.uiDataType);
            act_content->image_publisher=this->create_publisher<sensor_msgs::msg::Image>(pub_name+"/image_raw", 32);
            act_content->camera_info=this->create_publisher<sensor_msgs::msg::CameraInfo>(pub_name+"/camera_info", 32);
            act_content->Image=sensor_msgs::msg::Image(rosidl_runtime_cpp::MessageInitialization::SKIP);
            
          }
          act_content->init=true;
        } else {
          act_content->lines++;
          if(pLine->sCSI2Header.sShortHeader.uiDataType < 0x10){
            /* short packages do not have any payload */
            pLine++;
          } else {
            if(act_content->length != pLine->sCSI2Header.sLongHeader.uiWordCount && act_content->length_odd!=pLine->sCSI2Header.sLongHeader.uiWordCount && act_content->length_odd==0) {
              if( act_content->lines&1 && weird_line_behavior(pLine->sCSI2Header.sShortHeader.uiDataType) ) {
                /* ok, there are some data types with different line lengths - handle those! */
                act_content->length_odd=pLine->sCSI2Header.sLongHeader.uiWordCount;
              } else {
                RCLCPP_ERROR(this->get_logger(), "CSI2 line lenght is not constant! Check if source is a image and if everythin is configured correctly: %d - %d", act_content->length, pLine->sCSI2Header.sLongHeader.uiWordCount);
                /* we'll use the new length, but abort processing (to not spam to much errors). if the new length is correct, the next frame will prove this,
                  else we'll just get another error */
                act_content->length = pLine->sCSI2Header.sLongHeader.uiWordCount;
                return false;
              }
            }

            const SCSI2RawLineFooter_t* pEnd = (const SCSI2RawLineFooter_t*)&((const uint8_t*)(pLine+1))[pLine->sCSI2Header.sLongHeader.uiWordCount];
            pLine = (const SCSI2RawLineHeader_t*)(pEnd+1);
          }
        }
      }
      return true;
    }

    bool convert_frame(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr& msg, port_context& context ) {
      uint32_t line_count=0;
      SCSI2RawLineCollection_t* pCSI2 = (SCSI2RawLineCollection_t*)&msg->data[msg->offset_to_payload];
      const SCSI2RawLineHeader_t* pLine=(const SCSI2RawLineHeader_t*)(pCSI2+1);
      
      /* iterate through all CSI2 lines until we reach the end of our memory block */
      while( (void*)pLine < (void*)&msg->data[msg->data.size()] ) {
        /* grab something shorter based on virtual channel and data type */
        auto act_content=&context.frame_content[pLine->sCSI2Header.sShortHeader.uiVirtualChannel][pLine->sCSI2Header.sShortHeader.uiDataType];
        line_count++;
        if(line_count > msg->number_lines) {
          RCLCPP_ERROR(this->get_logger(), "more CSI2 lines found in frame than stated in header - seems to be something corrupt/wrong configured.");
          return false;
        }
        if(!act_content->init) {
          RCLCPP_ERROR(this->get_logger(), "CSI2 frame content changed between frames - check if your config is valid and produces stable data.");
          return false;
        }
        
        if(pLine->sCSI2Header.sShortHeader.uiDataType < 0x10) {
          /* if we get a end-of-frame (0x01) or a new start-of-frame (0x00) for a virtual channel, we close what we have and publish this */
          if(pLine->sCSI2Header.sShortHeader.uiDataType==0x00 || pLine->sCSI2Header.sShortHeader.uiDataType==0x01 ) {
            for(auto& dt : context.frame_content[pLine->sCSI2Header.sShortHeader.uiVirtualChannel]) {
              if(dt.second.image_pos) {
                dt.second.Image.height=dt.second.image_lines;
                dt.second.Image.width=dt.second.width;
                dt.second.Image.encoding = csi2_type_to_ros2(dt.first);
                dt.second.Image.header.frame_id=msg->header.frame_id + "_vc_" + std::to_string(pLine->sCSI2Header.sShortHeader.uiVirtualChannel) + "_dt_" + std::to_string(dt.first);
                dt.second.Image.header.stamp.sec=msg->header.stamp.sec;
                dt.second.Image.header.stamp.nanosec=msg->header.stamp.nanosec;
                dt.second.Image.step=dt.second.length;
                dt.second.Image.is_bigendian=false; /* not sure how it's used */
                
                /* fill camera_info accordingly */
                dt.second.CamInfo.header.frame_id=dt.second.Image.header.frame_id;
                dt.second.CamInfo.header.stamp=dt.second.Image.header.stamp;
                dt.second.CamInfo.height=dt.second.Image.height;
                dt.second.CamInfo.width=dt.second.Image.width;

                /* publish it */
                dt.second.image_publisher->publish(dt.second.Image);
                dt.second.camera_info->publish(dt.second.CamInfo);

                /* reset for the next image */
                dt.second.image_pos=0;
                dt.second.image_lines=0;
                dt.second.Image=sensor_msgs::msg::Image(rosidl_runtime_cpp::MessageInitialization::SKIP);
              }
            }
          } 
          /* short packages do not have any payload */
          pLine++;
        } else {
          if(!act_content->Image.data.size()) {
            act_content->Image.data.resize(act_content->lines*act_content->length);
          }

          memcpy(&act_content->Image.data.data()[act_content->image_pos], (pLine+1), pLine->sCSI2Header.sLongHeader.uiWordCount);
          act_content->image_pos+=pLine->sCSI2Header.sLongHeader.uiWordCount;
          act_content->image_lines++;
          if(act_content->length != pLine->sCSI2Header.sLongHeader.uiWordCount) {
            RCLCPP_ERROR(this->get_logger(), "CSI2 line lenght is not constant! Check if source is a image and if everythin is configured correctly: %d - %d", act_content->length, pLine->sCSI2Header.sLongHeader.uiWordCount);
            /* we'll use the new length, but abort processing (to not spam to much errors). if the new length is correct, the next frame will prove this,
                else we'll just get another error */
            act_content->length = pLine->sCSI2Header.sLongHeader.uiWordCount;
            return false;
          }
          const SCSI2RawLineFooter_t* pEnd = (const SCSI2RawLineFooter_t*)&((const uint8_t*)(pLine+1))[pLine->sCSI2Header.sLongHeader.uiWordCount];
          pLine = (const SCSI2RawLineHeader_t*)(pEnd+1);
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
        meta_cache[InstanceChannel].init=true;
      }
      convert_frame(msg, meta_cache[InstanceChannel]);
    }
    rclcpp::Subscription<mdi_msgs::msg::MdiCsi2Frame>::SharedPtr subscription_;
    instance_channel_mapping meta_cache;
};

#include <rclcpp_components/register_node_macro.hpp>
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MdiConverterNode)