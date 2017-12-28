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
[image6]: ./misc_images/wc.png
[image7]: ./misc_images/IK.png
[image8]: ./misc_images/seq_1.png
[image9]: ./misc_images/seq_2.png
[image10]: ./misc_images/seq_3.png
[image11]: ./misc_images/seq_4.png
[image12]: ./misc_images/inv_r.png
[image13]: ./misc_images/final_thetas.png
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

Here are the individual Transformation matrices between joints: 
```python
T0_1

⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢                          ⎥
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1  0.75⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦

T1_2

⎡cos(q₂ - 0.5⋅π)   -sin(q₂ - 0.5⋅π)  0  0.35⎤
⎢                                           ⎥
⎢       0                 0          1   0  ⎥
⎢                                           ⎥
⎢-sin(q₂ - 0.5⋅π)  -cos(q₂ - 0.5⋅π)  0   0  ⎥
⎢                                           ⎥
⎣       0                 0          0   1  ⎦

T2_3

⎡cos(q₃)  -sin(q₃)  0  1.25⎤
⎢                          ⎥
⎢sin(q₃)  cos(q₃)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1   0  ⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦

T3_4

⎡cos(q₄)   -sin(q₄)  0  -0.054⎤
⎢                             ⎥
⎢   0         0      1   1.5  ⎥
⎢                             ⎥
⎢-sin(q₄)  -cos(q₄)  0    0   ⎥
⎢                             ⎥
⎣   0         0      0    1   ⎦

T4_5

⎡cos(q₅)  -sin(q₅)  0   0⎤
⎢                        ⎥
⎢   0        0      -1  0⎥
⎢                        ⎥
⎢sin(q₅)  cos(q₅)   0   0⎥
⎢                        ⎥
⎣   0        0      0   1⎦

T5_6

⎡cos(q₆)   -sin(q₆)  0  0⎤
⎢                        ⎥
⎢   0         0      1  0⎥
⎢                        ⎥
⎢-sin(q₆)  -cos(q₆)  0  0⎥
⎢                        ⎥
⎣   0         0      0  1⎦

T6_G

⎡1  0  0    0  ⎤
⎢              ⎥
⎢0  1  0    0  ⎥
⎢              ⎥
⎢0  0  1  0.303⎥
⎢              ⎥
⎣0  0  0    1  ⎦
```

and here is the complete homogeneous transform from `base_link` to `gripper_link`:

```python
T0_G

⎡((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅c
⎢                                                                             
⎢((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅c
⎢                                                                             
⎢                               (-sin(q₅)⋅sin(q₂ + q₃) + cos(q₄)⋅cos(q₅)⋅cos(q
⎢                                                                             
⎣                                                                             

os(q₂ + q₃))⋅cos(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅sin(q₆
                                                                              
os(q₂ + q₃))⋅cos(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅sin(q₆
                                                                              
₂ + q₃))⋅cos(q₆) - sin(q₄)⋅sin(q₆)⋅cos(q₂ + q₃)                               
                                                                              
0                                                                             

)  -((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁
                                                                              
)  ((-sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) + sin(q₄)⋅cos(q₁))⋅cos(q₅) - sin(q₁)⋅sin(q₅
                                                                              
                                   (sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos
                                                                              
                                                                              

)⋅cos(q₂ + q₃))⋅sin(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅cos
                                                                              
)⋅cos(q₂ + q₃))⋅sin(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅cos
                                                                              
(q₂ + q₃))⋅sin(q₆) - sin(q₄)⋅cos(q₆)⋅cos(q₂ + q₃)                             
                                                                              
   0                                                                          

(q₆)  -(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(
                                                                              
(q₆)  (-sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) + sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(
                                                                              
                        -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)  
                                                                              
                                                 0                            

q₅)⋅cos(q₂ + q₃)  -0.303⋅sin(q₁)⋅sin(q₄)⋅sin(q₅) + 1.25⋅sin(q₂)⋅cos(q₁) - 0.30
                                                                              
q₅)⋅cos(q₂ + q₃)  1.25⋅sin(q₁)⋅sin(q₂) - 0.303⋅sin(q₁)⋅sin(q₅)⋅sin(q₂ + q₃)⋅co
                                                                              
                                                          -0.303⋅sin(q₅)⋅cos(q
                                                                              
                                                                              

3⋅sin(q₅)⋅sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄) - 0.054⋅sin(q₂ + q₃)⋅cos(q₁) + 0.303⋅co
                                                                              
s(q₄) - 0.054⋅sin(q₁)⋅sin(q₂ + q₃) + 0.303⋅sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃) + 1.5⋅
                                                                              
₄)⋅cos(q₂ + q₃) - 0.303⋅sin(q₂ + q₃)⋅cos(q₅) - 1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂)
                                                                              
                                          1                                   

s(q₁)⋅cos(q₅)⋅cos(q₂ + q₃) + 1.5⋅cos(q₁)⋅cos(q₂ + q₃) + 0.35⋅cos(q₁)⎤
                                                                    ⎥
sin(q₁)⋅cos(q₂ + q₃) + 0.35⋅sin(q₁) + 0.303⋅sin(q₄)⋅sin(q₅)⋅cos(q₁) ⎥
                                                                    ⎥
 - 0.054⋅cos(q₂ + q₃) + 0.75                                        ⎥
                                                                    ⎥
                                                                    ⎦
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

![equation][image6]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Using the Law of Cosines and trignometry, and with the help of the following image, we derive the inverse kinematics equations up to the WC using a geometrical approach. Different perspectives are used in order to get all three angles `theta_1`, `theta_2` , and `theta_3`.

Here is the section of the code that deals with calculating the thetas, based on the derived equations from the following figure:

```python
            

            a = 1.501            
            b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow(WC[2] - 0.75, 2))
	    c = 1.25

            alpha = acos((-a*a + b*b + c*c)/(2*b*c))
            beta = 	acos((a*a - b*b + c*c)/(2*a*c))
            gamma = acos((a*a + b*b - c*c)/(2*a*b))

            rest_alpha = atan2((WC[2] - 0.75), sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
	    
	    theta1 = atan2(WC[1], WC[0])
            theta2 = pi/2 - alpha - rest_alpha
            theta3 = pi/2 - (beta + 0.036)
```


![inverse_kinematics][image7]


Next we get the rotation matrix from WC to gripper using the following equations:

![inv_R][image12]

And with the resulting matrix we are able to extract `theta_4`, `theta_5`, and `theta_6` using the following equations, where we extract elements from the matrix so as to satisfy the `theta = arctan2(sin(theta), cos(theta))` :

![final_thetas][image13]



The code implementation for the previous concepts can be seen here: 
```python
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            
            R3_6 = R0_3.inv('LU') * R_EE[:-1, :-1]
            
            pprint(simplify(R3_6))
            
            R0_3 = R0_3.evalf(subs = {q1: theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3.inv('LU') * R_EE[:-1, :-1]

            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

With the help of the walkthrough, Slack, and the lessons in the Pick and place project, i implemented all the above mentioned steps in sequence, the same way i implemented in the coding quizzes, but plus the new parts (inverse rotation matrix, Wrist center position vector, etc).

Some parts i did myself and some parts i followed the walkthrough , although i made sure i understood it before implementing.

Using the combo of ROS, Gazebo, and Rviz helped me understand more the various aspects of being a robotics software engineer. Although the projects are to be considered simple as we only built a small part of code. Building a 2 DOF manipulator from scratch would be really cool. Node initialization, services, pub-subs and URDFs and connecting the parts together would also be a great thing to do, which plan on, as soon as i finish term 1. 

On the other hand, areas of improvement for my project would be:

- using numpy instead of sympy for faster computation, 
- implementing an algorithm to better pick the solutions for the inverse kinematics problems
- saving computed matrices as pickle files

Regarding issues i faced, i felt lost at first because i did not know where to start, and where to implement the specific parts of the code, but as soon as i watched the walkthrough and worked with the debug code, i was able to understand and visualize what i should be doing.


Here's a sequence of images during the place motion during one Pick and Place cycle:

![seq_1][image8]
![seq_2][image9]
![seq_3][image10]
![seq_4][image11]



