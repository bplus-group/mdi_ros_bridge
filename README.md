# MDI Node for ROS2

## Overview
This package contains the support for the (M)easurment (D)ata (I)nterfaces - MDI - from b-plus GmbH. Those devices are basically data converter for arbitrary interfaces to Ethernet. Arbitrary interfaces are typically something like MIPI CSI2, HSSL, LVDS, etc... 
This node wraps around the regular API for those devices.

> ### Note<br>
> This package is intended as a quick start for the intergration of MDI devices. The acutal evaluation of the grabbed data has to be done on purpose - and maybe for performence reasons also inside this node.

## Content

### mdi_msgs
Provides all the messages used by the MDI Node(let).

### mdi_node 
Contains the ROS2 wrapper implementation around the MDI Rx API as a seperate ROS2 Node.

### mdi_nodelet
Contains the (almost) identical ROS2 wrapper implementation from __mdi_node__ as ROS2 Component (aka Nodelet).

### third_party
Contains the actual MDI Rx API for various platforms (ok, right now just linux x86_64). The while the Windows counterpart is available as binary, currently no ROS2 setup on Windows is at hand.

## Installation

Clone the repository
```
git clone <url_to_do>
```

-optional- update the API binaries (typically, those are ABI stable)<br>
-> replace the binaries in third_party folder

### Build as nodelet
```
colcon build
or
colcon build --cmake-clean-cache
```



### Build as node
```
AS_NODE= colcon build
or
AS_NODE= colcon build --cmake-clean-cache
```

If not already done, source ROS2
```
$ . ./ros2_galactic/install/local_setup.bash
```

Now build the node
```
$ colcon build
```

Source your node
```
$ . ./mdi_node/install/local_setup.bash
```

Start it
```
$ ros2 run mdi_node mdi_rx_node
```