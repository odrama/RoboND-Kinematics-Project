## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/robot_analysis.jpg
[image5]: ./misc_images/DH_table.jpg
[image6]: ./misc_images/wc_position.jpg
[image7]: ./misc_images/IK.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

By following the supplied examples in the Kinematics lessons, and using the Denavit-Hartenberg parameter convention, which is a systematic approach for describing coordinate frames relative to each other using just 4 parameters, i was able to fully derive the forward kinematics for the KR210 robot.

![Robot Analysis using the DH Convention][image4]


Link lengths and joint positions were extracted from the `URDF` supplied within the project's repo.
A simple summary of the variables would be:

`alpha`: The twist angle between a joint's axis and the preceding joint's axis (be it axis of rotation or translation)
`a`: Distance between a joint axis and the axis of the preceding one along the common normal
`theta`: Angle between the the common normal of a joint and the common normal of the preceding joint
`d`: Distance between the common normal of a joint and that of the one preceding it

![DH Parameter Table][image5]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The following code snippet includes a function `transform_matrix()` that takes in the four DH parameters and returns a transformation matrix between the related frames.

```python

	def transform_matrix(alpha, a, d, q):
			T = Matrix([[             cos(q),            -sin(q),        0,          a],
				[ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
				[ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
				[                   0,                   0,         0,               1]])
	
			return T

	T0_1 = transform_matrix(alpha0, a0, d1, q1).subs(s)
	T1_2 = transform_matrix(alpha1, a1, d2, q2).subs(s)
	T2_3 = transform_matrix(alpha2, a2, d3, q3).subs(s)
	T3_4 = transform_matrix(alpha3, a3, d4, q4).subs(s)
	T4_5 = transform_matrix(alpha4, a4, d5, q5).subs(s)
	T5_6 = transform_matrix(alpha5, a5, d6, q6).subs(s)
	T6_G = transform_matrix(alpha6, a6, d7, q7).subs(s)
```
We then multiply the transformations from the `base_link` to the `end_effector` in order to get the total transformation between them.

```python
	T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```
Using a sequence of Extrinsic rotations, we calculate the rotation matrix between the `base_link` and the `end_effector`

```python

	R_z = Matrix([[cos(y), -sin(y), 0, 0], 
	    [sin(y), cos(y), 0, 0], 
	    [0, 0, 1, 0],
	    [0, 0, 0, 1]]) # Yaw
	R_y = Matrix([[cos(p), 0, sin(p), 0], 
	    [0, 1, 0, 0], 
	    [-sin(p), 0, cos(p), 0], 
	    [0, 0, 0, 1]]) # Pitch
	R_x = Matrix([[1, 0, 0, 0], 
	    [0, cos(r), -sin(r), 0], 
	    [0, sin(r), cos(r), 0], 
	    [0, 0, 0, 1]]) # Roll
      
      
	R_EE = R_z * R_y * R_x
  ```
  
We then correct it with another rotation matrix, as the frame assigment implemented using the DH method is different from the assignmentsi n the `URDF` file for the KR210.

```python
	R_corr = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
	R_EE = R_EE * R_corr
```

The translation part of the Homogenous transform between them is obtained directly through the code.


And since the position vector of the Gripper w.r.t base is equal to the sum of the position vector of the Wrist Center w.r.t base plus the position vector of the Gripper w.r.t base, all we have to do is use the following equation, since by now, we have all the unknowns, in order to get the osition vector of the Gripper w.r.t base

![alt_text][image6]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Using the Law of Cosines and trignometry, and with the help of the following image, we derive the inverse kinematics up to the WC using a geometrical approach. Different perspectives are used in order to get all three angles `theta_1`, `theta_2` , and `theta_3`


![alt_text][image7]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


