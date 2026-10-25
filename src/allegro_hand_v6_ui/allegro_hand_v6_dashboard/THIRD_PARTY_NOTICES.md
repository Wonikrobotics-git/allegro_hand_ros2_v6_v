# Third-party notices

`allegro_hand_v6_dashboard` uses the following system-provided libraries. The
package does not vendor or statically link them.

| Component | License |
| --- | --- |
| Qt Core, Gui, Widgets and SVG runtime plugin | LGPL-3.0-or-later or GPL |
| ROS 2 `rclcpp`, `std_msgs`, `visualization_msgs` | Apache-2.0 |
| RViz (`rviz_common`, `rviz_rendering`, default plugins) | BSD-3-Clause-Clear |
| OGRE rendering engine | MIT |
| Assimp | BSD-3-Clause |
| yaml-cpp | MIT |

Binary distributors must preserve the applicable license texts and notices.
When Qt binaries are redistributed, distributors must also satisfy the LGPL,
including allowing library replacement/relinking and providing the
corresponding Qt source or a compliant written offer. This project intentionally
uses shared Qt libraries and does not use GPL-only Qt modules.
