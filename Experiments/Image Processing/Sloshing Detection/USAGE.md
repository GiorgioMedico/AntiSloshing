# Code Usage and Configuration Flags

This document provides a quick reference for running the main processing script and understanding its configurable flags.

---

## How to Use

To run the main processing script, simply execute it (e.g., `run Sloshing_Detection.m`). The specific behavior of the script is controlled by the flags detailed below, which are typically set at the beginning of the main script file.

---

## Configuration Flags

Adjust these flags at the top of your main script to control how the code executes and what outputs it generates.

### General Operation Flags

* `record_synchro `
    * Set to **`true`** if you want to **save the synchronized video streams** to disk. If **`false`**, videos are processed but not saved.
* `record_BW`
    * Set to **`true`** if you want to **save the segmented (black and white) video streams** to disk. This is useful for reviewing the segmentation quality.
* `inRange`
    * Set to **`true`** to enable the **HSV color range selection**. The segmentation will be done for hue, saturation, and value. If **`false`**, the script will use the predefined RGB red channel.
* `downsampling`
    * Set a value greater than `1` (e.g., `2`, `5`, `10`) to process only a fraction of the total frames. For example, `downsampling = 2` processes every second frame, which can significantly **speed up execution** for long videos.
* `inRange_open`
    * Set to **`true`** to apply a **morphological opening operation** immediately after HSV segmentation. This helps refine the segmented mask by removing small, isolated noise particles and smoothing boundaries. This flag is only effective if `inRange` is also set to **`true`**.
* `auto_remove_frames`
    * Set to **`true`** to enable an **automatic video trimming mode**. A preview window will appear, allowing you to easily define and remove unwanted frames from the beginning and end of the video sequence.
* `manual_remove_frames`
    * Set to **`true`** to enable **manual frame removal**. This provides more granular control, allowing you to specify exact frame numbers to skip or process, useful for very precise trimming or bypassing corrupted sections.

### Debug and Output Flags

These flags control the real-time visualization of intermediate results and the saving of analytical data and figures.

* `show_hsv`
    * Set to **`1`** (true) to display the **interactive HSV segmentation window** during the color range selection process (if `inRange` is `true`).
* `show_edge`
    * Set to **`1`** (true) to visualize the **detected liquid edge/contour** on the video frames as the code processes each frame.
* `show_mask`
    * Set to **`1`** (true) to visualize the **segmented binary mask** (the mask overlay showing the detected liquid) during execution.
* `show_reprojection`
    * Set to **`1`** (true) to visualize the **3D point reprojection** onto the 2D camera frames. This overlays the reconstructed 3D points back onto the original images, helping to verify the accuracy of the 3D reconstruction process.
* `save_fig`
    * Set to **`1`** (true) to automatically **save generated figures** (e.g., debug visualizations, plots) to disk.
* `save_mat`
    * Set to **`1`** (true) to **save important `.mat` files** containing processed data (e.g., calculated 3D points) for later analysis.

---

## Code Execution Flow

When you run the main script, here's a brief, step-by-step overview of what happens, directly influenced by the flags you've set:

1.  **Video Writer Initialization:**
    * `VideoWriter` objects for saving the segmented (black and white) videos are prepared if `record_BW` is set to **`true`**. This ensures the writers are ready when needed later in the loop.
2.  **Synchronization & Initial Recording:**
    * The script begins reading synchronized frames from the left and right video sources.
    * If `record_synchro` is **`true`**, these original synchronized frames are saved to a video file.
3.  **Video Loading & Trimming:**
    * The `VideoReader` objects are configured, and if `auto_remove_frames` or `manual_remove_frames` are active, the processing loop will be adjusted to read frames only from the specified, trimmed range of the video sequence.
    * If `auto_remove_frames` is **`true`** a pop-up window will appear, one video feed will be shown and the user will be prompted to specify initial and final times to be considered in the analysis.
4.  **General HSV Segmentation:**
    * If `inRange` is **`true`**, if `show_hsv` is **`true`** an interactive pop-up will guide you through setting general HSV color thresholds to identify the liquid, these values will be used for the sloshing peak detection, otherwise default values will be selected.
    * If `inRange_open` is also **`true`**, a morphological opening operation is applied to refine this initial segmented mask.
5.  **Diameter Detection & Specific HSV Segmentation:**
    * A **second, dedicated HSV segmentation process** occurs, specifically optimized for accurate diameter detection. This step might involve its own interactive tunin or a refinement of thresholds to precisely isolate the liquid's boundary for measurement.
6.  **Feature Detection (Frame by Frame) & Segmented Video Recording:**
    * Using the diameter-specific HSV results, a **binary mask** of the liquid is created (visualized if `show_mask` is **`true`**).
    * The **edge or contour** of this mask is then detected (visualized if `show_edge` is **`true`**).
    * If `record_BW` is **`true`**, these segmented (black and white) frames are saved in a video to disk as they are processed.
7.  **3D Reconstruction & Reprojection:**
    * The 2D detected features (e.g., the liquid's heighest points) from both camera views are used to reconstruct their corresponding **3D positions**.
    * If `show_reprojection` is **`true`**, the script displays the original camera frames with the **reprojected 3D points** overlaid, allowing visual verification of the 3D accuracy.
