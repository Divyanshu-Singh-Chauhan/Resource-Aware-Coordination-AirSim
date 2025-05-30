**ROS1 to ROS2 Migration Plan**

# **1. Preliminary Setup**

## **1.1. Set up ROS2 Environment**
1. Install the latest LTS version of ROS2 (e.g., Foxy or Galactic).
2. Install necessary dependencies:

```bash
sudo apt update
sudo apt install ros-<ros2-distro>-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

## **1.2. Create ROS2 Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## **1.3. Install Dependencies**
- Verify that dependencies (e.g., AirSim, Unity) are ROS2-compatible.
- Develop ROS2-compatible wrappers if necessary.

# **2. Transitioning ROS1 Code to ROS2**

## **2.1. ROS2 Package Conversion**
- Rename `CMakeLists.txt` and `package.xml` for ROS2 compatibility.
- Replace ROS1 dependencies with ROS2 equivalents.

**Example CMakeLists.txt for ROS2:**
```cmake
cmake_minimum_required(VERSION 3.5)
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_robot_node src/my_robot_node.cpp)
ament_target_dependencies(my_robot_node rclcpp std_msgs)

install(TARGETS my_robot_node DESTINATION lib/${PROJECT_NAME})

ament_package()
```

**Example package.xml for ROS2:**
```xml
<?xml version="1.0"?>
<package format="3">
    <name>my_robot_package</name>
    <version>0.0.1</version>
    <description>A simple robot package in ROS2</description>
    <maintainer email="email@example.com">Maintainer Name</maintainer>
    <license>Apache 2.0</license>
    <depend>rclcpp</depend>
    <depend>std_msgs</depend>
</package>
```

## **2.2. ROS2 Node Communication Transition**
- Replace ROS1 `Publisher`/`Subscriber` with ROS2 equivalents.
- Example migration:

**ROS1 Publisher:**
```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
```

**ROS2 Publisher:**
```cpp
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub =
    this->create_publisher<std_msgs::msg::String>("chatter", 10);
```

## **2.3. Service and Action Server Transition**

**ROS1 Service:**
```cpp
ros::ServiceServer service = nh.advertiseService("my_service", &MyClass::callback, this);
```

**ROS2 Service:**
```cpp
rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
    this->create_service<std_srvs::srv::SetBool>("my_service", std::bind(&MyClass::callback, this, std::placeholders::_1, std::placeholders::_2));
```

# **3. Transitioning AirSim Integration**
- Ensure AirSim has a ROS2-compatible client or develop one.
- Modify AirSim launch files:

**ROS1 Launch File:**
```xml
<launch>
    <node name="airsim_node" pkg="airsim_ros_pkgs" type="airsim_ros_node" output="screen"/>
</launch>
```

**ROS2 Launch File:**
```xml
<launch>
    <node name="airsim_node" pkg="airsim_ros2_pkgs" exec="airsim_ros_node" output="screen"/>
</launch>
```

# **4. Transitioning Unity Integration**
- Use `ROS-TCP-Connector` for ROS2-compatible Unity communication.
- Modify Unity scripts to use ROS2 messages.

**Example Unity ROS2 Publisher (C#):**
```csharp
using RosMessageTypes.Std;
using RosCSharp;

public class UnityPublisher : MonoBehaviour
{
    private RosSocket rosSocket;
    private string topicName = "/unity_topic";
    private Int32Msg msg = new Int32Msg();
    
    void Start()
    {
        rosSocket = new RosSocket("ws://localhost:9090");
        rosSocket.Advertise<std_msgs.Int32>(topicName);
    }

    void Update()
    {
        rosSocket.Publish(topicName, msg);
    }
}
```

# **5. Modify Launch Files**

**ROS1 Launch File:**
```xml
<launch>
    <node name="my_robot" pkg="my_robot_package" type="robot_node" output="screen"/>
</launch>
```

**ROS2 Python Launch File:**
```python
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_node',
            name='my_robot',
            output='screen',
        ),
    ])
```

# **6. Isolated Testing During Migration**

## **6.1. Testing ROS2 Node**
```bash
colcon build --packages-select my_robot_package
source install/setup.bash
ros2 run my_robot_package robot_node
```

## **6.2. Testing Communication**
```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello World'"
ros2 topic echo /chatter
```

## **6.3. Testing AirSim Integration**
```bash
ros2 node list
ros2 topic list
```

## **6.4. Testing Unity Integration**
- Use Unity’s built-in debugging tools to verify message exchange.

# **7. Finalizing the Migration**

## **7.1. Integration Testing**
- Ensure all components (ROS2 nodes, AirSim, Unity) work together.
- Verify multi-robot communication and interactions.

## **7.2. Code Cleanup**
- Remove ROS1 dependencies.
- Ensure ROS2-compatible syntax and structures.

# **8. Sharing with Contributors**

### **Migration Summary**
- **Step 1:** Set up ROS2 environment.
- **Step 2:** Convert ROS1 packages to ROS2.
- **Step 3:** Adapt AirSim to ROS2.
- **Step 4:** Modify Unity integration.
- **Step 5:** Convert launch files to Python.
- **Step 6:** Test each component independently.
- **Step 7:** Conduct full system integration testing.
- **Step 8:** Finalize and share the migration.

This document serves as a guide for contributors to follow during the migration process.

