# MDI Node and Nodelet for ROS2

## Overview
This package contains the support for the (M)easurement (D)ata (I)nterfaces - MDI - from b-plus technologies GmbH. Those devices are basically data converter for arbitrary interfaces to Ethernet. Arbitrary interfaces are typically something like MIPI CSI2, HSSL, LVDS, etc... 
This node wraps around the regular API for those devices.

> __Note__<br>
> This package is intended as a quick start for the integration of MDI devices. The actual evaluation of the grabbed data has to be done on purpose - and maybe for performance reasons also inside this node.

## Content

- __mdi_msgs__<br>
Provides all the messages used by the MDI Node(let).

- __mdi_daq_converter__<br>
Provides the facilities to convert raw DAQ frames into ROS2 compatible images. Currently only YUV422-8bit is supported, tough. However feel free to add other conversions.


- __mdi_node__<br>
Contains the ROS2 wrapper implementation around the MDI Rx API as a separate ROS2 Node.
Additionally a nodelet variant is available as well as a dummy publisher, which reads a raw DAQ Frame from disk and keeps on publishing it (for development purposes).


- __mdi_lib__
Contains the actual MDI Rx API for various platforms (ok, right now just linux x86_64). Windows is as of yet unsupported.

## Quick Start
We have both, a node and a nodelet - whatever fits the your approach most. From the wrappers point of view, the difference is next to not existing.

First, clone the repository
```
~$ git clone https://github.com/bplus-group/mdi_ros_bridge.git ~/ros2_ws/src
```

> __optional__<br>
update the API binaries (those are ABI stable) by replacing the binaries in mdi_lib folder




0. source your ros2 installation, if not already done
   ```
   ~$ cd ros2_galactic
   ~/ros2_galactic$ . ./install/local_setup.sh
   ```

1. build and source the MDI message definitions

   ```
   ~$ cd ros2_ws
   ~/ros2_ws$ colcon build
   ~/ros2_ws$ . ./install/local_setup.sh
   ```

### As nodelet

2. launch the nodelet
   ```
   ~$ ros2 launch mdi_node mdi_nodelet.launch.py
   ```

### As node


2. launch the node
   ```
   ~$ ros2 run mdi_node mdi_rx_node
   ```
## Provided Topics

> __Note__<br>
This is subject to change.

- __/mdi/rxapi/status__<br>
This is a status from the API itself, like number of received frames, errors, bandwidth and so on.

- __/mdi/raw_daq__<br>
The RAW frame is published when it's not a CSI2 oder Status frame from the MDI - it is expected to add some project dependent processing here.

- __/mdi/csi2_frame<br>
The CSI2 frame contains the image as transmitted via CSI2. It includes timestamps for every CSI2 line and also the meta data for every line. The line payload needs to be filtered and converted to whatever format is sufficient.

- __/mdi/status__<br>
The MDI Status frame contains a collection of status and performance values from the MDI as a large JSON object. It is provided periodically (every ~5s or so) to be included in recordings for later inspection (e.g. if something was not as expected).

## Architecture

### General notes
- The MDI RX API is available for both, Windows and Linux. The current node(let) implementation is for Linux, but has some beginning implementations for Windows - however, treat Windows as not supported, yet.

- The node and nodelet share the very same source file. The differences in the code are toggled by the define __AS_NODELET__.

- The wrapper is for the most parts identical (or at least similar) to the "rx_sampleapp" provided as example by the MDI RX API.

- The MDI RX API should be built and deployed by the node(let) build process, as we load the API dynamically. Why? Because we deliver a installation-less API to make version switching and deploying of a "full" application easier. 

- The node(let) binary (or shared object) will facilitate the binary constructor and destructor to load the MDI RX API dynamically.

### Received frame handling
- We register custom memory manager for the MDI RX API, so we get a finer control. On allocation, we already provide the memory of a mdi_msgs::msg::Mdirawframe (a preallocated std::vector).

- Upon completion of receiving a frame, the API provides us the same frame. On "freeing" the frame, the API will not only cleanup it's own meta data for this reception, but also calls the free of the provided memory manager. In this case, we ignore the frame itself on purpose, as the mdi_msgs::mdi::Mdirawframe will be cleaned up by other means.

- Getting data from the API and publishing is currently in the exact same thread context - as it is assumed, that publishing will de-couple threads anyway. If this is an issue, we should switch to a thread-decoupling ourselves.

- The reception thread currently identifies some of the more common MDI data types (like its status and CSI2 data frames) and provides different publishers for this. Depending on the use case we can either extend this to prepare real images (or other data equivalents) here or add a separate node, which will offload this. Please provide feedback!

### DAQ frame conversion ###
- The conversion tries to convert RAW CSI2 data into usable ROS2 images. This is easier for some data types and more complex for others. Right now we support YUV422-8bit, which is pretty standard and almost identical to ROS2. Other data types, like RAW-types are more complex, as you might need additional information about the Beyer-pattern, which is not part of the CSI2 standard. Or, if the RAW data in fact contains a RADAR image instead. <br>
However feel free to use this as a starting point for the conversion.

- The MDI might split image frames into separate DAQ frames, especially if the camera is using virtual channels. Typically this can be handled by a proper configuration, but the converters is able to fit those split frames together, anyway. 

- The converters creates an information tree of the data types. We split the following:<br>
  __MDI Instance -> MDI Channel -> Virtual CSI2 Channel -> CSI2 Data Type__<br>
  In this way we cover the following parts of the CSI2 standard:<br>
  -> __Virtual Channels__ which have their own separate framing (begin/end).<br>
  -> __Virtual Channel Interleaving__ where different channels are mixed in between the lines.<br>
  -> __Data Type Interleaving__ when multiple images with the same virtual channel are contained in the same CSI2 frame.<br>

- The converters creates a separate publisher for every image type in the following scheme:<br>
  mdi/instance__\<num>/port_\<port>/vc_\<virtual_channel>/dt_\<csi2_data_type_num> <br>
  eg: mdi/instance_0/port_1/vc_0/dt_30/image_raw

- Additionally, the converters add the typical camera_info used for cv_bridges along with the published image. <br>
  eg: mdi/instance_0/port_1/vc_0/dt_30/camera_info
