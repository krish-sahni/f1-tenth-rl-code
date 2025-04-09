# *ECE484 Final Project: Starter Code Guide*
There is 2 different starter code available for you:
##  *I. Vicon Pure Pursuit Tracker*
As shown in the F1-Tenth field trip, this code uses a pure pursuit controller to track a trajectory defined by a fixed set of waypoints. The localization is provided by the lab's Vicon cameras and is sent to the vehicle through the ViconMavlink. Please contact the TA/CA to set this up for you.

The primary file for running the vicon tracker code is located in
*src/f1tenth_control/vicon_control/scripts/vicon_tracker_pp.py*

All starter code is placed in this scripts directory
This vicon tracker code follows what is described in the MP2 documentation where a lookahead point is chosen from the waypoints and the steering angle is computed.

### *Tuning*
The steering angle can be tuned with the variable 
python
k

and the lookahead distance can be tuned with
python
self.look_ahead

The speed can also be adjusted with
python
self.drive_msg.drive.speed
The waypoints are currently loaded in from 
python
'vicon_control/xyhead_demo_pp.csv'.

They may not correspond with the lane exactly, so it is recommended to re-record the waypoints by moving the car around the track with a cart and perodically saving the local coordinates from vicon_bridge.py.


## *Running the Code*

### *Launch Commands*

Open each command in a *separate terminal window*, and ensure to source the workspace by running the commands in your project directory (you only need to chmod initially):

cd ~/Project_Root_Directory

chmod -R 777 ./

#### *Build Workspace (not necessary if WS is placed in nx directory)*

catkin_make


#### Terminal 1: Racecar Remote Launch

source devel/setup.bash

roslaunch racecar teleop.launch

#### Terminal 2: Vicon Bridge

source devel/setup.bash

python3 vicon_bridge.py

#### Terminal 3: Vicon Tracker Code

source devel/setup.bash

rosrun vicon_control/vicon_tracker_pp.py

---

*Note:* Each command runs in a dedicated terminal window.



## *II. Vision Based PID*

This section is designed as a high-level guide for you to understand the main components and code structure for the *lane detection* code ( using *color thresholding* and a *PID controller* to follow lanes. 

*Note*: This version of lane detection is more archaic, as it relies on straightforward color thresholding. You are free to keep it simple or adapt it into a more advanced technique (e.g. building on your MP1) if you wish. This code was tested during the night, so you will probably have to modify aspects of it to get the vehicle to move smoothly around the track. Throughout this guide, we will frequently point you to two primary code files of interest that you should start modifying:

- *lane_detection.py*
- *vision_lanefollower_pid.py*

In the sections below, we describe the general idea of how the lane detection pipeline works, the PID controller that uses the detected lane information, and the key code blocks you may need to modify for your own experiments.

----

### *Model Architecture*

This pipeline focuses on using *color thresholding* to isolate lane markings in an image. The resulting binary image is then used to calculate the lateral steering error for controlling the vehicle. The approach can be summarized as follows:

1. *Capture image from the camera*.  
2. *Apply color threshold* (to highlight the yellow lane markings).  
3. *Pre-process* this binary image to reduce noise.  
4. *Apply a perspective transform* to get a top-down “bird’s-eye” view of the lanes.  
5. *Compute the lane position* and derive a steering error with respect to the vehicle’s center.  
6. *Use a PID controller* to reduce the steering error and follow the lane.

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/a0df3d44-c91e-4f1d-96f9-e7f655b06274" alt="Project Pipeline">
      </td>
    </tr>
    <tr>
      <td align="center">
        Project Pipeline
      </td>
    </tr>
  </table>
</div>


As mentioned, this is an older approach to lane detection. The methods here (especially thresholding) may need to be tuned or improved depending on lighting or real-world changes. You are encouraged to experiment and extend these methods, or even adapt your MP1 logic if desired.

----

## *Lane Detection*

Below we outline the functions in *lane_detection.py* that perform the core of our vision-based lane detection. You will find these functions fully implemented in the starter code, but reading and understanding how they work is essential for any changes or improvements you might make.


### *Color Threshold*

The *color_thresh* function is responsible for filtering out pixels that match a certain range of *HLS* (Hue, Lightness, Saturation) values. 


    def color_thresh(self, img, thresh=(100, 255)):
        """
        Filters out yellow pixels from image.
        Depending on the lighting condition, the static color mask may not work well.
        It could be helpful to keep track of lighting condition that works best with your car and codebase,
        or you can modify this value.
        """
        # ... code that converts to HLS and applies threshold ...
        return yellow_mask

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/07c333c2-13e2-45b6-b711-1f80e6125f62" alt="Original Image" width="300">
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/8ab51937-1e3b-40e3-8aad-5006d54b86b8" alt="Color Threshold Image" width="300">
      </td>
    </tr>
    <tr>
      <td align="center">Original Image</td>
      <td align="center">Color Threshold Image</td>
    </tr>
  </table>
</div>


In our project, we assume the lane lines are *yellow*, so we provide two HLS ranges that capture most yellow tones.

    ranges = [
        (20, 35, 100, 255, 70, 255),
        (20, 35, 130, 160, 40, 60)
    ]

These ranges dictate what is considered “yellow.” If your environment or track has different lighting conditions, or a slightly different hue of yellow, you may want to tweak these numbers.


### *Pre-Processing of Binary Image*

The *getBinaryImage* function further cleans up the binary mask from *color_thresh*. Small specks and noise can cause false positives, so we remove them by applying morphological operations such as remove_small_objects.

    def getBinaryImage(self, img):
        """
        Post-processing the result from color_thresh to remove noise
        """
        # 1. Apply color_thresh to get an initial binary mask
        # 2. Remove small objects (morphological operation)
        # 3. Return a clean binary image for further processing
        return binaryImage

In practice, you may not need to alter this much if it works well with your environment, but understanding how morphological removal or dilation might help is beneficial.


### *Perspective Transform*

The *perspective_transform* function better estimates lane curvature and vehicle position by transforming our image into a *“bird’s-eye”* view. This makes it easier to measure lateral offsets and run polynomial fits for the PID controller.


    def perspective_transform(self, img, verbose=False):
        """
        Bird's eye view transformation of the image
        """
        # 1. Specify source points (src) around the region of interest
        # 2. Specify destination points (dst) in a top-down layout
        # 3. Use cv2.getPerspectiveTransform(...) and cv2.warpPerspective(...)
        return warped_img, M, Minv, src, dst

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/07c333c2-13e2-45b6-b711-1f80e6125f62" alt="Original Image" width="300">
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/6aa07e43-627b-4ee1-abc7-aab348073258" alt="Color Threshold Image" width="300">
      </td>
    </tr>
    <tr>
      <td align="center">Original Image</td>
      <td align="center">Birdseye View Image</td>
    </tr>
  </table>
</div>


### *Gradient Thresholding*

In this section, we introduce an optional *gradient_thresh* function that you could implement in your *lane_detection.py* file to detect lane lines based on gradient information using Sobel filters. While not strictly required, using gradient thresholding in combination with color thresholding can sometimes yield better results, especially in scenarios where color alone might not sufficiently isolate lane lines. Note that this function is not implemented and it is up to you code it, but we will provide detailed pseudocode to get you started.

    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Gradient Thresholding to detect lane edges
        """
        # 1. Convert the image to gray scale
        # 2. Apply Gaussian blur to reduce noise
        # 3. Use cv2.Sobel() to compute gradients along the X and Y axes
        # 4. Use cv2.addWeighted() (or another approach) to combine the gradient results
        # 5. Convert each pixel to uint8, then apply threshold to create the binary image
        return binaryImage

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/07c333c2-13e2-45b6-b711-1f80e6125f62" alt="Original Image" width="300">
      </td>
      <td align="center">
        <img src="https://github.com/user-attachments/assets/e677e6df-2733-4dac-ab9f-bc78b38d690b" alt="Gradient Threshold Image" width="300">
      </td>
    </tr>
    <tr>
      <td align="center">Original Image</td>
      <td align="center">Gradient Threshold Image</td>
    </tr>
  </table>
</div>

*Hint:* You can upload the original image to Google CoLab and unit test your gradient function.

----

## *PID Controller*

A *PID (Proportional–Integral–Derivative)* controller is used to convert the lane detection output (steering error) into a meaningful control signal (steering command). The objective is to minimize the lateral error between the center of the car as seen through the perspective transformed camera and the center of the lane.

Below is a quick overview, with pointers to relevant code.

*Where to look if you have starter code A:*:
- pid_controller.py – The main PID logic is defined here.
- vision_lanefollower_pid.py – This file integrates the PID logic with the lane detection outputs.
- lane_detection.py – Where the steering error is calculated (in detection(self, img) function).
*Where to look if you have starter code B:*:
- controller.py – The main PID logic is defined here. 
- vision_PID_lanefollower. – This file calls the execute function defined in controller.py that integrates the PID logic with the lane detection outputs .
- errorgen.py – Where the steering error is calculated (in detection(self, img) function).

## *How the PID Uses the Lane Detection: Starter Code A (scroll bellow for Starter Code B)*

In *lane_detection.py*, you’ll find the function:

    def detection(self, img):

        # 1. Preprocess image to get binary and bird's-eye view.
        # 2. Fit lane lines and compute steering error.
        # 3. Annotate image and return the annotated and bird's-eye view images.
        
        return combine_fit_img, bird_fit_img

Essentially, this function:
1. *Converts* the raw camera image to a binary bird’s-eye view.  
2. *Fits* lane lines to polynomials.  
3. *Computes* the difference between your car’s “center column” and the lane’s “lookahead column.”  
4. *Stores* this difference in self.steering_error, which the PID controller will use.

#### *PID Camera Parameters*

The constants used to set a “lookahead” row and a “center” reference are defined here:

    self.lookahead_row = 345
    self.lookahead_col = 0
    self.center_col = 640//2 + 40
    self.steering_error = 0.0

- *lookahead_row*: The row in the image (from the top) that we consider as a forward-looking reference.  
- *lookahead_col*: This value gets updated based on the polynomial fit. Initially zero.  
- *center_col*: Defines where we believe the camera’s center is. By default, if the image width is 640, then 320 is the center, but we might shift by +40 to align with the camera’s perspective on the car.  
- *steering_error*: The difference between the lane’s center at the lookahead row and the actual camera center column.

Feel free to play with these values if your lane detection is offset or if you want a different “lookahead” distance.

---

### *PID Gains*

Inside *vision_lanefollower_pid.py* (where the PID is integrated with the main loop), the following parameters are crucial in how aggressively or smoothly the car responds:

    self.Kp = 0.34
    self.Ki = 0.0
    self.Kd = 0.24

- *Kp*: Proportional gain (scales with immediate error).  
- *Ki*: Integral gain (accumulates past error).  
- *Kd*: Derivative gain (predicts future trend of the error).

Tweak these for better performance. If the vehicle oscillates too much, try lowering Kp or increasing Kd. If the vehicle fails to center itself, you might need a higher Kp or a small Ki term. This file also houses the main function, which is responsible for publishing the steering and speed ROS messages through the AckermannDriveStamped topic.

----
## *How the PID Uses the Lane Detection: Starter Code B (scroll up for Starter Code A)*

In *errorgen.py*, you’ll find the function:

    def detection(self, img):

        # 1. Preprocess image to get binary and bird's-eye view.
        # 2. Fit lane lines using the function line_fit(self, binary_warped)
        #    Note: line_fit annotates the image and return the annotated and bird's-eye view images.
        # 3. Compute steering error

Essentially, this function:
1. *Converts* the raw camera image to a binary bird’s-eye view.  
2. *Fits* lane lines to polynomials.  
3. *Computes* the difference between your car’s “center column” and the lane’s “lookahead column.”  
4. *Stores* this difference in self.error, which the PID controller will use.

#### *PID Camera Parameters*

The constants used to set a “lookahead” row and a “center” reference are defined here:

    # To store previous waypoints
    self.previouswaypoints = None
    self.previouserror = None
    self.error = None

    # parameters for error generation
    self.lookaheaddist = 0.7
    self.offset = 25
    self.midpoint = 160

- *lookaheaddist*: The forward-looking reference is calculated such that 
    length = len(waypoints)
    point = length - round( length * self.lookaheaddist).  
- *offset*: Shift to align with the camera’s perspective on the car.
- *midpoint*: Defines where we believe the camera’s center is.  
- *error*: The difference between the lane’s center at the lookahead  and the actual camera center.

Feel free to play with these values if your lane detection is offset or if you want a different “lookahead” distance.

---

### *PID Gains*

Inside *vision_lanefollower_pid.py* (where the PID is integrated with the main loop), the following parameters are crucial in how aggressively or smoothly the car responds:

    self.KP = 0.0
    self.KI = 0.0
    self.KD = 0.0
    self.SCALE_FACTOR = 0

- *Kp*: Proportional gain (scales with immediate error).  
- *Ki*: Integral gain (accumulates past error).  
- *Kd*: Derivative gain (predicts future trend of the error).
- *SCALE_FACTOR*: scales the error term.

Tweak these for better performance. If the vehicle oscillates too much, try lowering Kp or increasing Kd. If the vehicle fails to center itself, you might need a higher Kp or a small Ki term. This file also houses the main function, which is responsible for publishing the steering and speed ROS messages through the AckermannDriveStamped topic.

---

### *Running the Code:*

#### *Launch Commands*

Open each command in a *separate terminal window*, and ensure to source the workspace by running the commands in your project directory:

cd ~/Project_Root_Directory

chmod -R 777 ./

#### *Build Workspace (not necessary if WS is placed in nx directory)*

catkin_make

##### Terminal 1: Racecar Camera Launch

source devel/setup.bash

roslaunch racecar sensors.launch

##### Terminal 2: Racecar Remote Launch

source devel/setup.bash

roslaunch racecar teleop.launch

##### Terminal 3: Lane Following PID Control

source devel/setup.bash

rosrun vision_control vision_PID_{A or B}/vision_lanefollower_pid.py

##### Terminal 4: Calcutate Error (starter code B only)


source devel/setup.bash

rosrun vision_control vision_PID_B/errorgen.py

---

*Note:* Each command runs in a dedicated terminal window.

---


## *First Steps*

1. *Get the code running*. Ensure your environment is set up, and that the basic thresholding + perspective transform pipeline works. When the code is running, your car should roughly follow the yellow line on the track.

2. *Review the main functions*. Understand how each piece (threshold, transform, detection, PID) comes together. You can then experiment with your own ideas, or even build a more advanced approach on top of it. If you prefer, you can also start your own code from scratch, but this starter setup is there to help. Feel free to look at other files in the project; they are all commented.

You should now have a decent overview of how the lane detection module is organized and where the PID controller fits in. Good luck, and remember that everything in these files is fair game for improvement or complete replacement. Just be sure you understand the existing flow before making changes.

## *Credits*

Vicon controller is written by Hang Cui

Vison-PID Starter Code A is written by Hyoungju Lim, Vikram Battalapalli, Don Lazatin, Cheng Zhu 

Vision-PID Starter Code B is written by Tanvi Kulkarni, Harish Kumar Balaji, Vikram Raj Nagoor Kani, Manan Hitesh Maheshwari

