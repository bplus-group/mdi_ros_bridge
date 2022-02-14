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

typedef std::unordered_map<uint16_t, port_context> instance_channel_map;

/*
  Our converter separates the individual CSI2 lines based on the sending MDI (instance) and the physical
  port of the MDI (channel), the virtual channel of the CSI2 data and the actual data type.

  virtually:
  MDI
   |------ Instance 0
   |          |---------- Channel 0 (ChannelMapping)
   |          |               |--------- VC 0 (VCMapping)
   |          |               |           |--------- Data Type (DataTypeMapping)


  actual: we combine instance and channel as "instance_channel_map" as the additional effort to manage those two 
  creates more writing overhead than it solves.

  instance_channel_map
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


class MdiConverterNode : public rclcpp::Node {
  public:
    MdiConverterNode(const rclcpp::NodeOptions &option);

    virtual ~MdiConverterNode() { }



  private:
    
    rclcpp::Subscription<mdi_msgs::msg::MdiCsi2Frame>::SharedPtr subscription_;

    instance_channel_map meta_cache;
    
    /* we have some CSI2 data types with interleaving line lengths which need more effort to validate a common line length */
    bool detect_inconsistent_line_lengths(uint8_t dt);

    /* some CSI2 data types will map directly to a ROS2 type, others might need more "adjustment" */
    const char* csi2_type_to_image_encoding(uint8_t dt);

    /* depending on the data type, we need to derive the actual pixels from the number of bytes */
    uint32_t convert_line_length(uint8_t dt, uint32_t line, uint16_t num_bytes);

    bool parse_frame(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr& msg, port_context& context);

    bool convert_frame(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr& msg, port_context& context);

    void topic_callback(const mdi_msgs::msg::MdiCsi2Frame::SharedPtr msg);

    };

#include <rclcpp_components/register_node_macro.hpp>
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(MdiConverterNode)
