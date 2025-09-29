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


