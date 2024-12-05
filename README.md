# Lane Detection Project

## Description
This project implements a lane detection algorithm using edge detection and the Hough Transform to identify road lane boundaries. The algorithm processes a video frame by frame, detects lane lines, and overlays them on the video output with a green trapezoid highlighting the drivable region.

## Files Included
1. `variables.mat` - Contains pre-defined variables for the Region of Interest (ROI) required by the script.
2. `project_video.mp4` - Input video containing the driving scenario to be processed.

## Prerequisites
- MATLAB installed on your computer.

## How to Run the Code
1. **Upload Required Files:**
   - Upload `variables.mat` and `project_video.mp4` into the same directory where you will be working in MATLAB.

2. **Run the Code:**
   - Open the `lane_detection_code.m` file.
   - Copy the entire code from `lane_detection_code.m` and paste it into the MATLAB Command Window.
   - Press `Enter` to execute the script.

3. **Output Video:**
   - The processed video with lane detection will be saved in the same directory as `Result.avi`.

## Notes
- Ensure the `variables.mat` and `project_video.mp4` files are correctly uploaded into the directory before running the script.
- The output video (`Result.avi`) will show detected lane lines with red overlays and a green trapezoid highlighting the drivable region.

## Expected Output
- The program will process each frame of `project_video.mp4`, overlay lane lines, and save the final processed video as `Result.avi`.
