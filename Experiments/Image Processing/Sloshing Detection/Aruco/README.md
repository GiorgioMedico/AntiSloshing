# ArUco Camera Poses – Experiment Mapping

## Overview

Due to variations in experimental setup and different recording sessions, multiple localization runs were required. Each localization corresponds to a subset of videos based on their recording context and camera arrangement.

## File-to-Experiment Mapping

Two pose files are provided:

- **`poses_aruco_1.mat`**  
  This file contains camera localization data for videos with the following filename prefixes:  
  - `LE`  
  - `Tilt_LE`  
  - `TRD`  
  - `Tilt_TRD`

- **`poses_aruco_2.mat`**  
  This file corresponds to localization for videos with the prefixes:  
  - `RD`  
  - `Tilt_RD`

These prefixes indicate the specific experiments, and the use of multiple pose files ensures accurate tracking aligned with each setup.

## Notes

Ensure that when analyzing a video, the appropriate pose file is loaded to maintain consistency in the data interpretation.
