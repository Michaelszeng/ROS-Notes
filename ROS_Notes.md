# ROS

### 16.485 Clipboard
 - `ln -s ~/path_to/pkg` (Run from `~/16.485-VNAV/vnav_ws/src`): creates a symbolic link of that package into the workspace folder `vnav_ws/src`

### Tools Clipboard
 - `colcon build --symlink-install --packages-select <package-name>` (Run from `vnav_ws`)
 - `source install/setup.bash`
 - `ros2 node list`
 - `ros2 topic list`
 - `ros2 topic echo <topic-name>` (prints topic messages to terminal)
 - `ros2 run rqt_graph rqt_grph`
 - `ros2 run rqt_plot rqt_plot`
 - `ros2 run rqt_tf_tree rqt_tf_tree`
 - `ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]`

<br />

 - `rviz2`
 - `rosbag?`

### Colcon Commands
- `VERBOSE=1 colcon build --event-handlers console_direct+`
 - Print out low-level build commands
- `colcon build --packages-up-to trajectory_generation`
 - Build package and its dependencies
- `colcon build --symlink-install`
 - Installed files are symlinked to the src directory, so editing launch files (or python files?) doesn’t require re-building
- `colcon clean packages / colcon clean workspace`
 - Removes build / install directories



<br /><br />

### File Structure
**Workspace**
 - Outermost directory level, contains packages
 - Contains tools to compile packages

**Packages**
 - "Atomic Unit" - i.e. the release item in ROS software
 - Contain nodes, libraries, config files, launch files, etc.
 - `package.xml` (Package Manifest File): author, license, dependencies, compilation flags, etc.

**Messages**: interprocess communications. Fields of messages defined in .msg files.

**Services**: request/reply interaction between processes. Reply and request data types defined inside `srv` folder of package.

Example structure:
```
.
└── two_drones_pkg
    ├── CMakeLists.txt
    ├── README.md
    ├── config
    │   └── default.rviz
    ├── launch
    │   └── two_drones.launch
    ├── mesh
    │   └── quadrotor.dae
    ├── package.xml
    └── src
        ├── frames_publisher_node.cpp
        └── plots_publisher_node.cpp
```


<br /><br />


### Making a Package
**Python**:
```
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs sensor_msgs -- <package-name>
```
`rclpy` is needed to use python with ROS. `std_msgs` is needed to publish or read messages from topics.


**CMake**:
```
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs tf2 tf2_ros -- <package-name>
```

**For Custom Messages**:
(which require `--build-type ament_cmake`), Add to CMakeLists.txt:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OpenSpace.msg"
)
```
Add to package.xml:
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```


### Running a Node
```
ros2 run <package_name> <node_name>
```

# C++

### Publisher Node
```C++
#include <string>
#include "rclcpp/rclcpp.hpp"  // Contains basic ROS function
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node {
  public:
    MinimalPublisher() : Node("minimal_publisher") {
      // publish messages w/type std_msgs::msg::String
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    void run() {
        auto message = std_msgs::msg::String();
        size_t count = 0;
        rclcpp::Rate r(1);  // Hz
        while (rclcpp::ok()) {
            message.data = "Hello, world! " + std::to_string(count++);
            publisher_->publish(message);
            r.sleep();
        }
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  MinimalPublisher pub;
  pub.run();
  rclcpp::shutdown();
  return 0;
}
```

### Subscriber Node
```C++
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber") {
      // Subscribe to "topic" of type std_msgs::msg::String, with queue size 10
      // Callback function topic_callback()
      subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    void topic_callback(const std_msgs::msg::String& msg) const {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // Node will keep running until killed
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  return 0;
}
```

### Launch Files (yaml)
Launch files located in `launch` folder in package.

```bash
ros2 launch <package_name> <launch_file>
```

Example:
```yaml
launch:

- node:
    pkg: minimal_test
    exec: publisher_node
    name: pub_node
    namespace: my_robot

- node:
    pkg: minimal_test
    exec: subscriber_node
    name: sub_node
    parameters:
        turtlename: 'turtle1'
        queue_size: 10
    remap:
    -   from: "topic"
        to: "/my_robot/topic"
```

Note `namespace: my_robot` of the first node; this means the node will be named `my_robot/pub_node` and the topics published by the node will be pre-pended with `my_robot/`.

How to read parameters:
```C++
turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
```

Note `remap: ...` in the second node; this simply renames "topic" to "/my_robot/topic" for that node. This `remap` feature can be useful to switch topic names quickly without modifying code, i.e. to run multiple non-conflicting instances of a robot, or to rename 3rd-party named topics.


### Transforms (`tf2_ros`)
2 types of tf2 nodes:
- Listener: listen to `/tf` topic and caches all data
- Broadcaster: publishes transforms between coordinate frames to `/tf` topic

Transforms are kept in tree structure; to compute relative transforms between 2 arbitrary frames, follow edges from one frame to the other and compose the transforms.

**Static vs Dynamic**: Static transforms typically broadcast once, defining fixed transform i.e. between camera and robot.

**Broadcasting a Static Transform in a Node**:
```C++
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// Create Broadcaster object as class attribute
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

// Construct the publisher in the constructor
tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

// Making and publishing a transform
// Static transforms usually published just ONCE (in constructor)
void make_transforms(double x, double y, double z, double r, double p, double y) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(r,p,y);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
}
```

<br />

**Commandline Tool for Broadcasting a Static Transform**
```Bash
# RPY
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --yaw yaw --pitch pitch --roll roll --frame-id frame_id --child-frame-id child_frame_id
# Quaternion
ros2 run tf2_ros static_transform_publisher --x x --y y --z z --qx qx --qy qy --qz qz --qw qw --frame-id frame_id --child-frame-id child_frame_id
```

**Broadcasting a Standard Transform in a Node**:
```C++
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// Create Broadcaster object as class attribute
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

// Construct the publisher in the constructor
tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

// Making and publishing a transform
// Usually called in a subscriber callback, i.e. when new sensor data is received
void make_transforms(double x, double y, double z, double r, double p, double y) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "<child-name>";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(r, p, y);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(t);
}
```

**Listening to Transforms**
```C++
#include <chrono>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

// Create Listener, buffer, and timer object as class attributes
rclcpp::TimerBase::SharedPtr timer_{nullptr};
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

// Construct the listener and buffer in the constructor
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// In constructor, set on_timer function to be called every second
timer_ = this->create_wall_timer(
    1s, [this]() {return this->on_timer();});

void on_timer() {
    // Store frame names in variables that will be used to compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames

    // VERSION 1: BLOCKS FOR UP TIMEOUT IF NO TRANSFORM IS AVAILABLE
    // Check if the transform is available within a timeout period
    // tf2::TimePointZero retrieves latest transform in buffer
    if (tfBuffer.canTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, rclcpp::Duration(std::chrono::milliseconds(100)))) {
        // The transform is available, now we can safely retrieve it
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
    } else {
        RCLCPP_WARN(this->get_logger(), "Transform not available between %s and %s", fromFrameRel.c_str(), toFrameRel.c_str());
    }

    // VERSION 2: DO NOT BLOCK, INSTEAD JUST RETURN IF NO TRANSFORMS AVAILABLE
    try {
        // tf2::TimePointZero retrieves latest transform in buffer
        t = tf_buffer_->lookupTransform(toFrameRel, 
                                        fromFrameRel,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    // Use transform data
    // t.transform.translation.x
    // t.transform.translation.y
    // ...
}
```
Note: the way tf buffers work is that they always hold all transforms from the last (default 10) seconds. `lookupTransform()` with time parameter `tf2::TimePointZero` will continue to return the same transform until either a new transform enters the buffer or 10 seconds has passed (and the buffer is emptied).

**Broadcasting a Frame**
```C++
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// Create Broadcaster, timer object as class attributes
rclcpp::TimerBase::SharedPtr timer_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

// In Constructor:
tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

auto broadcast_timer_callback = [this](){
    geometry_msgs::msg::TransformStamped t;

    // Construct new fixed frame named "carrot1" relative to "turtle1"
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
};
timer_ = this->create_wall_timer(100ms, broadcast_timer_callback);
```
Note: This code is near identical to a standard transform broadcaster, except the transform is fixed and we pick the name for the child frame.

Note: If you want the "carrot1" frame to move relative to "turtle1", simply set the `t.transform` values dynamically instead of setting them to constants.


## CMakeLists.txt
```bash
# Add each executable Node file
add_executable(node_name src/node_name.cpp)

# Add libraries
ament_target_dependencies(
   node_name
   rclcpp
   geometry_msgs
   tf2
   tf2_ros
)

# Tells ROS to install the node executables in the install/lib folder
install(TARGETS
   node_name
   DESTINATION lib/${PROJECT_NAME}
)
```


# Python

### Launch files
In `package_name/launch/launch_file_name.xml`:
```xml
<launch>
    <node pkg="final_challenge" exec="planner" name="planner" output="screen">
        <param from="$(find-pkg-share final_challenge)/config/sim_config.yaml"/>
    </node>
</launch>

```

If using a param file: (`config.yaml`):
```
planner:
  ros__parameters:
    odom_topic: "/odom"
    map_topic: "/map"
    scan_topic: "/scan"
    initial_pose_topic: "/initialpose"
trajectory_follower:
  ros__parameters:
    odom_topic: "/odom"
    drive_topic: "/drive"
```

Add to `setup.py`:
```python
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
```

Run `ros2 launch launch_file_name.xml` from `package_name/launch` folder

### Publisher Node

Place this file in `<package_name>/src`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        queue_size = 10
        self.publisher_ = self.create_publisher(Float32, 'my_random_float', queue_size)
        timer_period = 1 / 20  # 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = random.random() * 10.0  # Random float between 0 and 10
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add to `entry_points` `console_scripts` in `setup.py` (in package root dir):
```pythons
'simple_publisher = ros_exercises.simple_publisher:main',
```

### Subscriber Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'my_random_float',
            self.listener_callback,
            10)  # queue size 10
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float32, 'random_float_log', 10)  # queue size 10

    def listener_callback(self, msg):
        float_msg = msg.data
        log_msg = Float32()
        log_msg.data = math.log(float_msg)  # Compute the natural log
        self.publisher_.publish(log_msg)
        self.get_logger().info(f"Received: '{float_msg}', Published Log: '{log_msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Add to `entry_points` `console_scripts` in `setup.py` (in package root dir):
```pythons
'simple_subscriber = ros_exercises.simple_subscriber:main',
```

### ROS Parameters

Instead of needing to hardcode values, this is the same as providing arguments to the file when running or in launch file.

Add in `__init__()` function after `super().__init__()`:
```python
# Second param is default value (which also helps define parameter type)
self.declare_parameter('publish_topic', 'fake_scan')
self.declare_parameter('publish_rate', 1 / 20)
self.declare_parameter('angle_min', -(2/3) * math.pi)
self.declare_parameter('angle_max', (2/3) * math.pi)

self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
```

### Transforms
To write a node that listens to and/or publishes transforms:
```python
class SimplePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')

        # instantiate buffer that the listener will write to
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # broadcaster that will publish the transform
        self.br = tf2_ros.TransformBroadcaster(self)

        timer_period = 1 / 20  # 20Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            # get latest base_link_gt TF
            tf_world_to_robot: TransformStamped = self.tfBuffer.lookup_transform('world', 'base_link_gt',
                                                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().info('no transform from world to base_link_gt found')
            return
        
        ...

        # list of transforms to publish
        self.br.sendTransform([tf_world_to_left_camera, tf_left_camera_to_right_camera])
```

Helpful Utilty functions for converting to/from Numpy arrays and TFs:
```python
def tf_to_se3(self, transform: TransformStamped.transform) -> np.ndarray:
    """
    Convert a TransformStamped message to a 4x4 SE3 matrix 
    """
    q = transform.rotation
    q = [q.x, q.y, q.z, q.w]
    t = transform.translation
    mat = tf_transformations.quaternion_matrix(q)
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat

def se3_to_tf(self, mat: np.ndarray, time: Any, parent: str, child: str) -> TransformStamped:
    """
    Convert a 4x4 SE3 matrix to a TransformStamped message
    """
    obj = geometry_msgs.msg.TransformStamped()

    # current time
    obj.header.stamp = time.to_msg()

    # frame names
    obj.header.frame_id = parent
    obj.child_frame_id = child

    # translation component
    obj.transform.translation.x = mat[0, 3]
    obj.transform.translation.y = mat[1, 3]
    obj.transform.translation.z = mat[2, 3]

    # rotation (quaternion)
    q = tf_transformations.quaternion_from_matrix(mat)
    obj.transform.rotation.x = q[0]
    obj.transform.rotation.y = q[1]
    obj.transform.rotation.z = q[2]
    obj.transform.rotation.w = q[3]

    return obj
```

<br /><br />

# Archive

## Docker Setup

### Display forwarding

Once again, start `VcXsrv` by running XLaunch, and selecting "Disable Access Control".

In the dockerfile, add the following:
```
RUN apt-get update && apt-get install -y \
    x11-apps \
    libqt5widgets5
```

In `docker-compose.yml`, add the following:
```
environment:
    - DISPLAY=10.29.241.49:0
```

where you should use the IPv4 IP address of your own machine (which you can find using `ipconfig` command in powershell).


### Clipboardss
```Bash
# In Powershell
cd C:\Users\micha\OneDrive\Documents\School\mit_sem_4\6.4200\racecar_docker

docker compose up
...
docker compose down


# In Remote Terminal
sudo password: racecar@mit


rsync -avhe ssh racecar:~/racecar_ws/src ~/racecar_ws/src
```

Development: Attach to Remote Docker in VSCode.

If having display issues:
```Bash
export DISPLAY=<IP ADDRESS>:0.0
```
where you can find <IP ADDRESS> by running ipconfig in a command prompt (and use the IPv4 address under WSL)

<br />

```bash
build

# this is an alias refined in ~/.bashrc for the following:
colcon build --packages-select package_name
source install/setup.bash
```
