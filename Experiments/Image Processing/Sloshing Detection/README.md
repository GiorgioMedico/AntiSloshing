
# Image Processing Codebase for Liquid Sloshing Experiments

This repository is publicly available as a companion to the paper "**[Sloshing-Height Estimation for Liquid-filled Containers under Four-Dimensional Motions Including Spatial Translation and Rotation about a Fixed Direction: Modelling and Experimental Validation]**" by **R. Di Leva, S. Soprani, G. Palli, L. Biagiotti and M. Carricato**. This codebase and the accompanying video datasets were employed to validate the models developed in the aforementioned paper. We hope that these resources will be useful for replicating the obtained results and for fostering continuous improvement in liquid sloshing applications.

---

## Table of Contents
- [Project Overview](#project-overview)
- [Experimental Setup](#experimental-setup)
- [Algorithm Explanation](#algorithm-explanation)
  - [0. Cameras Localization](#0-cameras-localization)
  - [1. Video Synchronization](#1-video-synchronization)
  - [2. Video Pre-processing and Trimming](#2-video-pre-processing-and-trimming)
  - [3. Liquid Segmentation](#3-liquid-segmentation)
  - [4. Video Cropping](#4-video-cropping)
  - [5. Liquid Peak Detection](#5-liquid-peak-detection)
  - [6. 3D Peak Reconstruction](#6-3d-peak-reconstruction)
  - [7. Debug and Saving](#7-debug-and-saving)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
- [Data](#data)
- [AI Usage Aid](#ai-usage-aid)
- [License](#license)
- [Contact](#contact)
- [Citation](#citation)

---

## Project Overview

This repository provides the MATLAB code and supplementary data used for image processing and 3D reconstruction of liquid sloshing dynamics, as detailed in our research paper. The core functionality includes video synchronization, robust liquid segmentation, 2D peak detection from two camera views, and their subsequent triangulation into a single 3D coordinate representing the liquid's highest point. This experimental setup allows for precise measurement of sloshing height over time.

---

## Experimental Setup
The experiments were conducted using a cylindrical container with a radius $R = 49 \text{mm}$, filled with colored water to a static height of $h = 78 \text{ mm}$. Trajectories were performed by an industrial robot (Comau Smart Six). The robot's end-effector was equipped with two GoPro Hero8 cameras, placed with an angular offset of $90^\circ$ relative to each other around the container's axis. This camera arrangement was crucial to ensure comprehensive detection of the liquid peak, regardless of its position along the container wall.

---

## Algorithm Explanation

The image processing pipeline involves several key steps, executed sequentially for accurate 3D liquid peak detection:

### 0. Cameras Localization

The two cameras are localized via means of an Aruco marker, accurately positioned in the view of both cameras before the start of the experiments. Based on the camera parameters and the Aruco dimension the camera poses relative to the Aruco frame are computed and saved in a '.mat' file. The 'poses_aruco.mat' files used in the experiments are available in the 'Aruco/Poses' folder. Several sessions of experiments have been necessary to gather all video data, hence multiple '.mat' files are present. The video files corresponding to each '.mat' file are present in a 'README.md' file inside the 'Aruco' folder.   

**_Code Description for this step:_**
* **Script:** `aruco_detection.mlx`
* **Input:** `GoProRightParams_VideoMode.mat`, `videos_path`
* **Output:** `aruco_poses.mat` (Homogeneous matrices representing the position and orientation of the two GoPros in the Aruco frame)
* **Details:** This function leverages `readArucoMarker` to localize the GoPros.

### 1. Video Synchronization

The two video streams, recorded independently by the GoPros, are synchronized using their audio tracks. Cross-correlation between the audio signals of each camera is performed to find the lag that maximizes similarity, thereby determining the precise time delay between the two videos. This delay ensures that corresponding frames across both cameras capture the same instant in time.

**_Code Description for this step:_**
* **Script/Function:** `getDelay.m`
* **Input:** `left_video_path`, `right_video_path`
* **Output:** `tau` (scalar, delay in seconds)
* **Details:** This function leverages `audioread` to extract audio from video files and `xcorr` to compute cross-correlation between the left and right audio channels. The lag corresponding to the maximum correlation is then converted to a time delay using the known GoPro audio sample rate ($48 \text{ kHz}$). The average of delays from both stereo channels is used for `tau`.

### 2. Video Pre-processing and Trimming

After synchronization, the raw video streams are further processed to isolate the relevant motion and optimize for subsequent analysis.

* **Initial Synchronization Adjustment:** The `VideoReader` objects are initialized with `CurrentTime` adjusted by the calculated `tau` value, ensuring both videos start at their synchronized point.
* **Interactive Motion Trimming:** To focus on the specific period of liquid sloshing, a user-interactive preview is provided. This allows manual identification and selection of the precise start and end times of the relevant motion within the synchronized video segments. Frames outside this defined window are effectively excluded from further analysis. To select this option the `auto_remove_frames` flag needs to be set to `true`.

**Function:** `previewAndTrimVideo.m`
* **Input:** `videoObj_L`, `initial_startTime_L`, `videoObj_R`, `initial_startTime_R`
* **Output:** `final_startTime_L`, `final_endTime_L`, `final_startTime_R`, `final_endTime_R`
* **Details:** This function displays a single video (e.g., the left camera's feed) from its synchronized start. Users can pause playback and input the desired absolute start and end times for the motion. The function then calculates the corresponding start and end times for both videos, ensuring their synchronized trim for subsequent processing.

* **Manual Motion Trimming:** Alternatively the frames can be manually removed specifying the number of frames to remove at the start and at the end of each video. To select this option the `manual_remove_frames` flag needs to be set to `true`.

**Function:** `removeFrames.m`
* **Input:** `videoObj`, `startTime`, `numFramesToRemoveStart`, `numFramesToRemoveEnd`
* **Output:** `startTime`, `endTime`
* **Details:** This function removes the specified number of frames from the beginning and the end of both videos , ensuring their synchronized trim for subsequent processing.

### 3. Liquid Segmentation

For each frame within the trimmed video segments, the image is binarized to distinguish the colored liquid from the background.

* **Methodology:** The system offers two segmentation approaches:
    * **HSV Thresholding:** For colored liquid, this method leverages the Hue, Saturation, and Value (HSV) color space to isolate the liquid. This approach is highly effective in varying lighting conditions.
    * **Red Channel Binarization:** For cases where HSV might be less suitable or for specific color profiles, binarization based on a fixed threshold of the Red channel is available. Morphological operations (closing and opening) are applied to refine the binary mask, remove noise, and fill small gaps.

**_Code Description for this step:_**
* **Script:** `Sloshing_Detection.m` (Specifically, the `Range selection for segmentation` section)
* **Details:** The binarization selection is done in this portion, specifically the hsv 'inRange' values need to be selected.
    * **HSV:** When `inRange` is true, `rgb2hsv` is used, and pixels are segmented based on predefined or optionally user-provided `h_min`, `h_max`, `s_min`, `s_max`, `v_min`, `v_max` values.
    * **Red Channel:** When `inRange` is false, the red channel is extracted, `imbinarize` with a fixed `level` (e.g., `0.2`) is applied, followed by `imclose` and `imopen` with specified `strel` parameters (e.g., `disk`, 3 and 1 respectively) to clean the mask.
* **Interactive HSV Tuning:** You can also use `interactiveHsvSegmentation.m` to visually tune the HSV thresholds. To select this option you need to set to 'true' the flag 'show_hsv'.

### 4. Video Cropping

For each trimmed video segments, the size is cropped in order to isolate the liquid filled container and reduce the region of interest of the algorithm to avoid lateral regions that could negatively act on the automatic detection of the liquid surface and to shorten the execution times

* **Methodology:** if 'show_hsv' is 'true' the user is asked again to define the 'hsv' binarization parameters, this time for the container diameter selection. Indeed, the system will task the user to define the pixel height at which the diameter detection will be done. We recomend to select more aggressive 'hsv' parameters to correctly detect the container diameter, and we recomend a pixel height of 600. 

**_Code Description for this step:_**
* **Function:** `findDiameter.m`
* **Input:** `frame` (single video frame), 'D', 'h',`inRange` (boolean flag for HSV or Red channel method), `[optional_hsv_thresholds]`
* **Output:** `det_ind1', 'det_ind2' (the column pixels which define the start and end of the container)

### 5. Liquid Peak Detection

From each binarized frame, the frame is binarized and the liquid peak is detected. This involves searching for the highest black pixel, which corresponds to the highest point of the liquid surface in the image plane.

**_Code Description for this step:_**
* **Script:** `Sloshing_Detection.m`
* **Details:** In this code portion each frame is binarized and many vision operations are performed based on the active flags. In every case the frame is segmented as to select only the pixels in the corrected range, the biggest blob in the frame is selected while all the others are discarded, the liquid edge is then detected by means of a Canny Edge Detector, and finally the heighest row pixel is selected and stored, together with its original column index.

### 6. 3D Peak Reconstruction

The 2D coordinates of the detected peaks from each camera are then transformed into a single 3D coordinate in space

* **Camera Calibration:** This step relies on pre-calibrated camera intrinsic parameters, which define the camera's optical properties and distortion characteristics.
* **Line Computation:** For each camera, a line is computed connecting the optical center of the camera to the corresponding 2D peak pixel.
* **Triangulation:** The intersection of these two 3D lines (one from each camera) gives the absolute 3D coordinates of the true liquid peak. The vertical component of this 3D coordinate provides the experimental measure of the sloshing height at that instant.

**_Code Description for this step:_**
* **Script/Function:** `triangulate.m`
* **Details:** The `triangulate.m` MATLAB function returns the 3D point based on the two 2D points of each camera.

### 7. Debug and Saving

The `Sloshing_Detection.m` script makes available multiple flag variables in order to save and debug the algorithm results.
An extensive explanation of those variables is present in both the main script and in the `USAGE.md` file.

---

## Repository Structure
```
├── include/                           # Contains all MATLAB functions
│   ├── getDelay.m                     # Estimates audio delay between videos
│   ├── previewAndTrimVideo.m          # Interactive video preview and trimming
│   ├── findDiameter.m                 # Image binarization and diameter detection
│   ├── interactiveHsvSegmentation.m   # (Optional) Tool for tuning HSV thresholds
│   ├── removeFrames.m		             # Function to manually remove frames
│   ├── longest_interval.m	           # Detect longest pixel continuous pixel chain 
│   └── ...                            # Other utility functions
├── Aruco/...                          # Directory with Aruco videos and mat files
│   ├── Poses/			        
│   ├── Videos/
│   └── README.md		                   # File with explanation on Aruco mat files usage
├── Data/...                           # Plot figures and sloshing mat files
│   ├── mat/			        
│   └── Plots/
├── GoPro/...                          # Directory with stored videos
│   ├── GoPro_Right/
│   └── GoPro_Left/
├── Sloshing_Detection.m               # Main script to run the full pipeline
├── aruco_detection.mlx	               # Main script to obtain localized camera poses
├── GoProRightParams_VideoMode.mat     # GoPro cameras intrinsics parameters
├── README.md                          # This file
├── USAGE.md                           # Doc file explaining the flag variables and script usage
└── LICENSE                            # Licensing information
```
---

## Getting Started

Follow these instructions to set up the environment and run the image processing pipeline.

### Prerequisites

* MATLAB (R2024b or later recommended)
* MATLAB Image Processing Toolbox
* MATLAB Computer Vision Toolbox (for camera calibration and 3D reconstruction)
* MATLAB Audio Toolbox (if `audioread` is not directly supported by `VideoReader`)

### Installation

**Clone the repository:**
```bash
git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
cd your-repo-name
```

### Usage

1.  **Prepare your data:** Place your video files in the `GoPro/` children directories or update the `left_video_path` and `right_video_path` variables in `Sloshing_Detection.m`. Ensure your camera calibration files (`.mat` files) are also present.
2.  **(Optional) Localize the cameras:**
    Modify and execute the `aruco_detection.mlx` to obtain the mat file containing the camera poses.
    ```matlab
    aruco_detection
    ```
3.  **Adjust parameters (if needed):**
    * **Update Paths:** Update the folder paths and file names to the correct 'aruco_poses.mat' and video files.
    * **Flag Selection:** Modify the option flags to your desire.
    * **HSV Segmentation:** If the default HSV thresholds are not satisfactory for your liquid color, you can use the `interactiveHsvSegmentation.m` tool to find optimal values, then update them in or pass them as an optional argument.
4.  **Run the main script:**
    Execute the `Sloshing_Detection.m` to start the analysis.
    ```matlab
    Sloshing_Detection
    ```
    The script will guide you throughout its execution in the MATLAB command window. More in depth explanation of the script usage is present in **USAGE.md**.

---

## Data

The `GoPro/` folder contains the video clips (`.mp4`) used in the paper. These samples allow users to immediately test the pipeline. For your own experiments, replace these files with your own video and calibration data.

---
## AI Usage Aid

During the development of this codebase, generative AI tools were utilized to assist with various aspects of code refinement, and documentation. This included:

* **Code Structure and Best Practices:** Suggestions for organizing MATLAB functions, improving readability, and adhering to common coding conventions.
* **Function Design:** Aid in defining function interfaces, input/output arguments, and internal logic for specific tasks.
* **Documentation Generation:** Drafting and enhancing `README.md` and code documentation content, including code comments.

It is important to note that all AI-generated content was thoroughly reviewed, tested, and validated by the human authors to ensure correctness, accuracy, and adherence to the project's requirements. This acknowledgment is provided to promote transparency in our development process.

---

## License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## Contact


For any questions or inquiries, please contact:
-   **R. Di Leva**: [roberto.dileva@unibo.it]
-   **S. Soprani**: [simone.soprani2@unibo.it]

---

## Citation

If you use this codebase or the provided data in your research, please cite the accompanying paper:

```bibtex
@article{yourArticleKey,
  title={Sloshing-Height Estimation for Liquid-filled Containers under Four-Dimensional Motions Including Spatial Translation and Rotation about a Fixed Direction: Modelling and Experimental Validation},
  author={R. Di Leva, S. Soprani, G. Palli, L. Biagiotti and  M. Carricato},
  journal={Nonlinear Dynamics},
  year={2025},
  volume={Volume},
  pages={Pages},
  publisher={Publisher}
}
```