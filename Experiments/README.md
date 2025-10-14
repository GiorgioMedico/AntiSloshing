# Experiments

This folder contains code and instructions for the **experimental validation** of sloshing behavior. It includes:

- Image processing tools for extracting fluid motion metrics (sloshing height)
- Utilities for verifying and preparing joint trajectory `.csv` files for execution on a real COMAU robot

---

## 🤖 Real COMAU Robot Setup

The COMAU SmartSix robot located at **LAR** is controlled via **ROS**. The experimental procedure requires uploading and executing precomputed joint trajectories in `.csv` format.

### 🔁 Execution Pipeline

1. **Generate a `.csv` trajectory file** from your PC with:
   - **6 columns**, each corresponding to a joint
   - **`;` separator**
   - **500 Hz** sample frequency (i.e., time step of **0.002 s**)
   - ⚠️ **Strongly recommended**: Add at least **1 second (500 rows)** of idle configuration at the beginning of the trajectory to prevent abrupt motion due to communication latency.

2. **Power on the COMAU robot** and set it to its initial joint state.

3. Follow the execution procedure on the robot's controller PC:
   - See instructions in `~/ros/Comau_ws/README.md` on the robot's computer.
   - Use the appropriate ROS package (e.g., `csv_trajectory`) to execute the motion.

4. **Deploy your CSV**:
   - Copy it into the `trajectory/` folder on the robot
   - Update the `csv_reader.launch` (or equivalent launch file) to point to your CSV

---

## 🎥 Video Capture Guidelines

Video data is used for image-based analysis of sloshing using the detection pipeline in the [`Image Processing`](../Image%20Processing/Sloshing%20Detection/README.md) folder.

### 📷 Camera Setup (GoPro Hero 8)

To ensure high-quality, analyzable footage:

- **Check battery and storage** on both GoPro cameras
- Set mode to `Video`
  - Ensure **Linear mode** is enabled
  - Turn **HyperSmooth OFF**
- Use voice commands:
  - Start: `GoPro start recording`
  - Stop: `GoPro stop recording`
  - Verify that **both cameras** begin/stop recording as expected (look for flashing red light)

### 🎯 Marker Setup

- Position the **Aruco marker** vertically (Y-axis pointing up) with as little error as possible
- Ensure the marker is visible in **both** camera views
- Record a **video** (not just a photo) of the Aruco in a stationary position
- Remove the marker once the videos are acquired
- Place the liquid filled container in its position 

Note: Everytime the GoPro cameras are moved (e.g. to charge battery etc.) from the position of marker video acquisition, a new acquisition is needed.

Before motion begins:
- **Speak the trajectory name aloud** on the video to aid syncing and labeling

### 💡 Scene Setup Tips

- Minimize **light refraction or liquid glare** by adjusting lighting and wall positioning with respect to the container
- Ensure consistent camera angles across experiments
- Clearly separate the background to improve segmentation accuracy

---

## 📎 References

- Image processing and sloshing detection:  
  [`Image Processing/Sloshing Detection/README.md`](../Image%20Processing/Sloshing%20Detection/README.md)

- Robot controller startup guide:  
  `~/ros/Comau_ws/README.md` (on robot controller PC)

---

