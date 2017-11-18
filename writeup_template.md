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
[dh]: ./misc_images/dh.png
[theta123]: ./misc_images/figure_theta123.png
[result1]: ./misc_images/result1.png
[result2]: ./misc_images/result2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][dh]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3 
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

I define function to get homogeneous transform from dh parameter.
```
def homo_transform (alpha, a, d, theta):
    T = Matrix([[           cos(theta),           -sin(theta),           0,             a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                    0,                     0,          0,              1]])
    return T
```
Usage examples are as follows.
```
       T0_1 = homo_transform(alpha0, a0, d1, q1)

        T1_2 = homo_transform(alpha1, a1, d2, q2)

        T2_3 = homo_transform(alpha2, a2, d3, q3)

        T3_4 = homo_transform(alpha3, a3, d4, q4)

        T4_5 = homo_transform(alpha4, a4, d5, q5)

        T5_6 = homo_transform(alpha5, a5, d6, q6)

        T6_G = homo_transform(alpha6, a6, d7, q7)        
```

Generalized homogeneous transform between baselink and gripper link using only end-effector(gripper) pose is calculated as bellow.
```
Matrix([
[            cos(q1),            -sin(q1),            0,              a0],
[sin(q1)*cos(alpha0), cos(alpha0)*cos(q1), -sin(alpha0), -d1*sin(alpha0)],
[sin(alpha0)*sin(q1), sin(alpha0)*cos(q1),  cos(alpha0),  d1*cos(alpha0)],
[                  0,                   0,            0,               1]])
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

First, we calculate the position of the WC using the EE pose and position:
```
WC = Matrix([px, py, pz, 1]) - d7 * (T0_6 * Tcorr)[:, 2]
```
* px,py,pz is EE position relative to the base frame.
* d7 is link length from the WC to the EE.
* Tcorr is correction orientation difference between urdf versus dh convertion.
* T0_6 * Tcorr is Rotation of the EE relative to the base frame.

Next we calculate theta 1,2,3.

* theta1
Theta1 can be calculated like this.
```
theta1 = atan2(wc_y, wc_x)
```
wc_y,wc_x is WC's position calculated above.

* theta2 and 3
![alt text][theta123]
From the above figure theta2 can be calculated as following.
```
s1 = sqrt(wc_x ** 2 + wc_y ** 2) - a1 #distance between WC and basepoint minus distance between 2nd joint point and basepoint on xy cordinate
s2 = wc_z - d1 #distance between WC and basepoint minus distance between 2nd joint point and basepoint on z cordinate
s3 = sqrt(s1 ** 2 + s2 ** 2) 
s4 = sqrt(a3 ** 2 + d4 ** 2) 

beta1 = atan2(s2,s1)
cbeta2 = (a2 ** 2 + s3 ** 2) / 2 * a2 * s3 #cosine theorem
beta2 = atan2(sbeta2/cbeta2) #sbeta2 is calculate from cbeta2

cbeta3 = (a2 ** 2 + s4 ** 2) / a * a2 * s4 #cosine theorem
beta3 = atan2(sbeta3,cbeta3) #sbeta3 is calculate from cbeta3
beta4 = atan2(-a3,d4)

theta2 = pi/2 - beta1 - beta2
theta3 = pi/2 - beta3 - beta4
```

* theta4,5 and 6
I calculate the spherical wrist's rotation matrix by using this formula:
```
R3_6 = R0_3 ** -1 * R0_6 # ** -1 is calculate inverse matrix
```
With this rotation matrix and use tf transform function called [eular_from_matrix](http://docs.ros.org/jade/api/tf/html/python/transformations.html).
```
 theta4, theta5, theta6 = tf.transformations.euler_from_matrix(R3_6, axes='ryzx')
```
However, theta 4 and theta 5 required these additional calculations:
```
theta5 = theta5 - pi / 2
theta6 = theta6 - pi / 2
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

* Implimentation
Inverse Kinematics In the IK_server.py file, I implemented Inverse Kinematics as explained in the paragraph above. 
First I tried all calculations beforehand and performed calculation by substitution process, but I took a very long time to pre-calculate and abandoned it.
Since it took time to do simplify with a lot of symbols, we tried to numerically calculate by pre-assignment as much as possible to achieve high speed.

And just for fun, another example image:
![alt text][result1]
![alt text][result2]


