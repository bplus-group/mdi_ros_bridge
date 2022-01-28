# MDI Node and Nodelet for ROS2

## Overview
This package contains the support for the (M)easurment (D)ata (I)nterfaces - MDI - from b-plus GmbH. Those devices are basically data converter for arbitrary interfaces to Ethernet. Arbitrary interfaces are typically something like MIPI CSI2, HSSL, LVDS, etc... 
This node wraps around the regular API for those devices.

> __Note__<br>
> This package is intended as a quick start for the intergration of MDI devices. The acutal evaluation of the grabbed data has to be done on purpose - and maybe for performence reasons also inside this node.

## Content

- __mdi_msgs__<br>
Provides all the messages used by the MDI Node(let).

- __mdi_daq_converter__<br>
Provides the facilities to convert raw DAQ frames into ROS2 compatible images. Currently only YUV422-8bit is supported, tough. However feel free to add other conversions.

- __mdi_dummy_pub__<br>
For development purposes this publisher reads a dump of a DAQ Frame and publishes it over and over. It is basically a modification of the _mdi_nodelet_ for offline use.

- __mdi_node__<br>
Contains the ROS2 wrapper implementation around the MDI Rx API as a seperate ROS2 Node.

- __mdi_nodelet__<br>
Contains the (almost) identical ROS2 wrapper implementation from __mdi_node__ as ROS2 Component (aka Nodelet).

- __third_party__
Contains the actual MDI Rx API for various platforms (ok, right now just linux x86_64). The while the Windows counterpart is available as binary, currently no ROS2 setup on Windows is at hand.

## Quick Start
We have both, a node and a nodelet - whatever fit's the your approach most. From the wrappers point of view, the difference is next to not existing.

First, clone the repository
```
~$ git clone <url_to_do>
```

> __optional__<br>
update the API binaries (those are ABI stable) by replacing the binaries in third_party folder



### As nodelet

0. source your ros2 installation, if not already done
   ```
   ~$ cd ros2_galactic
   ~/ros2_galactic$ . ./install/local_setup.sh
   ```

1. build and source the MDI message definitions

   ```
   ~$ cd sw_lib_bplus_mdi_ros2/mdi_msgs
   ~/sw_lib_bplus_mdi_ros2/mdi_msgs$ colcon build
   ~/sw_lib_bplus_mdi_ros2/mdi_msgs$ . ./install/local_setup.sh
   ```

2. build the MDI receiver as nodelet / component
   ```
   ~$ cd sw_lib_bplus_mdi_ros2/mdi_nodelet
   ~/sw_lib_bplus_mdi_ros2/mdi_nodelet$ colcon build
   ~/sw_lib_bplus_mdi_ros2/mdi_nodelet$ . ./install/local_setup.sh
   ```

3. launch the nodelet
   ```
   ~$ ros2 launch mdi_nodelet mdi_nodelet.launch.py
   ```

### As node

0. source your ros2 installation, if not already done
   ```
   ~$ cd ros2_galactic
   ~/ros2_galactic$ . ./install/local_setup.sh
   ```

1. build and source the MDI message definitions

   ```
   ~$ cd sw_lib_bplus_mdi_ros2/mdi_msgs
   ~/sw_lib_bplus_mdi_ros2/mdi_msgs$ colcon build
   ~/sw_lib_bplus_mdi_ros2/mdi_msgs$ . ./install/local_setup.sh
   ```

2. build the MDI receiver as a node
   ```
   ~$ cd sw_lib_bplus_mdi_ros2/mdi_node
   ~/sw_lib_bplus_mdi_ros2/mdi_node$ colcon build
   ~/sw_lib_bplus_mdi_ros2/mdi_node$ . ./install/local_setup.sh
   ```

3. launch the nodelet
   ```
   ~$ ros2 run mdi_node mdi_rx_node
   ```
## Provided Topics

> __Note__<br>
This is subject to change.

- __/mdi/rxapi/status__<br>
This is a status from the API itself, like number of received frames, errors, bandwidth and so on.

- __/mdi/raw_daq__<br>
The RAW frame is published when it's not a CSI2 oder Status frame from the MDI - it is expeded to add some project dependent processing here.

- __/mdi/csi2_frame<br>
The CSI2 frame contains the image as transmitted via CSI2. It includes timestamps for every CSI2 line and also the meta data for every line. The line payload needs to be filtered and convertet to whatever format is sufficient.

- __/mdi/status__<br>
The MDI Status frame contains a collection of status and performance values from the MDI as a large JSON object. It is provided regulary (every ~5s or so) to be included in recordings for later inspection (e.g. if something was not as expected).

## Architecture

### General notes
- The MDI RX API is available for both, Windows and Linux. The current node(let) implementation is for Linux, but has some beginning implementations for Windows - however, treat Windows as not supported, yet.

- The node and nodelet share the very same source file. The differences in the code are toggled by the define __AS_NODELET__.

- The wrapper is for the most parts identical (or at least similar) to the "rx_sampleapp" provided as example by the MDI RX API.

- The MDI RX API should be build and deployed by the node(let) build process, as we load the API dynamically. Why? Because we deliver a installation-less API to make version switching and deploying of a "full" application easier. 

- The node(let) binary (or shared object) will facilitate the binary constructor and destructor to load the MDI RX API dynamically.

### Received frame handling
- We register custom memory manager for the MDI RX API, so we get a finer control. On allocation, we already provide the memory of a mdi_msgs::msg::Mdirawframe (a preallocated std::vector).

- Upon completion of receiving a frame, the API provides us the same frame. On "freeing" the frame, the API will not only cleanup it's own meta data for this reception, but also calls the free of the provided memory manager. In this case, we ignore the frame itself on purpose, as the mdi_msgs::mdi::Mdirawframe will be cleaned up by other means.

- Getting data from the API and publishing is currently in the exact same thread context - as it is assumed, that publshing will de-couple threads anyway. If this is an issue, we should switch to a thread-decoupling ourselves.

- The reception thread currently identifes some of the more common MDI data types (like its status and CSI2 data frames) and provides different publishers for this. Depending on usecase we can either extend this to prepare real images (or other data equvalents) here or add a seperate node, which will offload this. Please provide feedback!

### DAQ frame conversion ###
- The conversion tries to convert RAW CSI2 data into usable ROS2 images. This is easier for some data types and more complex for others. Right now we supprot YUV422-8bit, which is pretty standard and almost identical to ROS2. Other data types, like RAW-types are more complex, as you might need additional information about the Beyer-pattern, which is not part of the CSI2 standard. Or, if the RAW data in fact contains a RADAR image instead. <br>
However feel free to use this as a starting point for the converstion.

- The MDI might split image frames into separate DAQ frames, especially if the camera is using virtual channels. Typically this can be handled by a proper configuration, but the converter is able to fit those split frames together, anyway. 

- The converter creates an information tree of the data types. We split the following:<br>
  __MDI Instance -> MDI Channel -> Virtual CSI2 Channel -> CSI2 Data Type__<br>
  In this way we cover the following parts of the CSI2 standard:<br>
  -> __Virtual Channels__ which have their own separate framing (begin/end).
  -> __Virtual Channel Interleaving__ where different channels are mixed inbetween the lines.
  -> __Data Type Interleaving__ when multiple images with the same virtual channel are contained in the same CSI2 frame.

- The converter creates a separate publisher for every image type in the following scheme:<br>
  mdi/instance__\<num>/port_\<port>/vc_\<virtual_channel>/dt_\<csi2_data_type_num> <br>
  eg: mdi/instance_0/port_1/vc_0/dt_30
