# SlamBot: ROS 2 Navigation with Gazebo Simulation

---

## 📌 About the Project
This project implements a differential-drive robot (**SlamBot**) in **ROS 2** with **Ignition Gazebo** simulation, integrated with **Nav2** for point-to-point navigation.  
The setup focuses on achieving a proper **TF tree**, **odometry**, **laser scan integration**, and enabling **navigation with or without AMCL**.

---

## 📖 Project Description
- The robot is a 4-wheel differential drive bot with a LiDAR sensor.  
- Simulation runs in **Ignition Gazebo**, with ROS–Gazebo bridging using `ros_gz_bridge`.  
- The **main challenge solved** in this project was TF frame mismatch (`slambot/base_link` vs `base_link`) which blocked RViz visualization and Nav2 operation.  
- After fixing frame IDs, the robot can:
  - Publish correct odometry (`odom → base_link`)  
  - Broadcast LiDAR scans (`scan` in `lidar_link` frame)  
  - Navigate point-to-point using Nav2 with:
    - **Static TF (`map → odom`)** for testing  
    - **AMCL localization** for real-world-like navigation  

**Why this method?**  
- Ignition Gazebo provides physics-accurate simulation and integrates tightly with ROS 2.  
- Using `ros_gz_bridge` ensures that ROS 2 tools like Nav2, RViz2, and tf2 can directly work with simulation data.  
- This setup replicates a realistic robotics pipeline: **Simulation → Localization → Navigation**.  

---

## ⚙️ Step-by-Step Project Setup

### 1️⃣ Build the Workspace
```bash
mkdir -p ~/SlamBot_ws/src
cd ~/SlamBot_ws/src
git clone <your_repo_url>
cd ..
colcon build
source install/setup.bash
```

### 2️⃣ Launch Rviz 
```bash
ros2 launch slambot_description display.launch.py
```

![SlamBot_Rviz](https://github.com/SwashBuckler104/SlamBot_ws/blob/main/images/Slambot_description.png)

### 3️⃣ Launch Gazebo with Robot
```bash
ros2 launch slambot_gazebo gazebo.launch.py
```

![SlamBot_Gazebo](https://github.com/SwashBuckler104/SlamBot_ws/blob/main/images/Slambot_gazebo.png)

### 4️⃣ Run Navigation with Static TF
```bash
ros2 launch slambot_navigation navigation.launch.py
# for storing logs
ros2 launch slambot_navigation navigation.launch.py >  navlog.log 2>&1
```
**Once Rviz is launched in local/global_costmap follow the below Steps**
  **- First use 2D Pose Estimate for initial position set**
![SlamBot_Gazebo](https://github.com/SwashBuckler104/SlamBot_ws/blob/main/images/nav2.png)

  **- Use 2D Goal Pose for point to point navigation(more videos in images folder)**
[nav2_a.webm](https://github.com/user-attachments/assets/5566e7cd-89e5-4fbe-a895-b3e296c04338)


### 5️⃣ Verify TF Tree
```bash
ros2 run tf2_tools view_frames
```

**✅ Expected TF Tree:**
```bash
map → odom → base_link → wheels
                     ↳ lidar_link
```

---

## 📦 Dependencies  
- ROS 2 Humble / Iron (recommended)  
- Ignition Gazebo (Fortress / Garden)  
- Required Packages:  
  - `ros_gz_sim`  
  - `ros_gz_bridge`  
  - `nav2_bringup`  
  - `robot_state_publisher`  
  - `tf2_ros`  
  - `slam_toolbox` (if using mapping)

 ## Current Errros Need 
**As you can see the rqt graph the nav2 nodes are not getting activated automatically _- reasons can be some node in nav2 created error and everyhting stopped working ::**
```bash
# Configure first
ros2 lifecycle set /bt_navigator configure
ros2 lifecycle set /planner_server configure      #fails??

# Now activate
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /planner_server activate

# same for controller server
ros2 lifecycle get /controller_server
ros2 lifecycle set /controller_server activate


# Manual Testing
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {w:1}}}}"
ros2 topic echo /navigate_to_pose/_action/feedback

# rqt graph
ros2 run rqt_graph rqt_graph
```
