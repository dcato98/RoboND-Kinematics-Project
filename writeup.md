# Udacity Robotics Nanodegree
---

## Project #2: Kinematics Pick & Place

---


**Goals of the project:**  
1. Set up a ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the **src** directory of the ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform kinematic analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the [`IK_server.py`](https://github.com/dcato98/robotics-nd-p2/blob/master/kuka_arm/scripts/IK_server.py) file with inverse kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/kuka_kr210_forward_kinematics_diagram.jpg
[image2]: ./misc_images/kuka_kr210_inverse_kinematics_diagram.jpg
[image3]: ./misc_images/misc2.png

## Rubric Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup

#### 1. Provide a Writeup that includes all the rubric points and how you addressed each one.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is the diagram I drew for determining the DH parameters:

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

**Here is the DH parameter table for Kuka KR210, where link 0 is the base_link and link 7 is the gripper_link:**

i | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | -pi/2+q2
3 | 0 | 1.25 | 0 | q3
4 | -pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6 
7 | 0 | 0 | 0.303 | 0

These were determined by systematically labelling the links and joints, choosing axes for each joint, identifying non-zero `a` and `d` parameters, and looking up the link lengths in the KR210.urdf.zacro file. Note that John J Craig's convention for defining DH parameters was used. See [this page](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/87c52cd9-09ba-4414-bc30-24ae18277d24/concepts/83630844-9905-40c3-ae2d-ab1c4a315043) for further details on the DH convention followed.

**Individual transformation matrices:**

matrix row | base_link to joint_1 | joint_1 to joint_2 | joint_2 to joint_3 | joint_3 to joint_4
--- | --- | --- | --- | ---
0 | [cos(q1), -sin(q1), 0, 0] | [sin(q2), cos(q2), 0, 0.35] | [cos(q3), -sin(q3), 0, 1.25] | [cos(q4), -sin(q4), 0, -0.054]
1 | [sin(q1), cos(q1), 0, 0] | [0, 0, 1, 0] | [sin(q3), cos(q3), 0, 0] | [0, 0, 1, 1.5]
2 | [0, 0, 1, 0.75] | [cos(q2), -sin(q2), 0, 0] | [0, 0, 1, 0] | [-sin(q4), -cos(q4), 0, 0]
3 | [0, 0, 0, 1] | [0, 0, 0, 1] | [0, 0, 0, 1] | [0, 0, 0, 1]

matrix row | joint_4 to joint_5 | joint_5 to joint_6 | joint_6 to gripper_link
--- | --- | --- | ---
0 | [cos(q5), -sin(q5), 0, 0] | [cos(q6), -sin(q6), 0, 0] | [1, 0, 0, 0]
1 | [0, 0, -1, 0] | [0, 0, 1, 0] | [0, 1, 0, 0]
2 | [sin(q5), cos(q5), 0, 0] | [-sin(q6), -cos(q6), 0, 0] | [0, 0, 1, 0.303]
3 | [0, 0, 0, 1] | [0, 0, 0, 1] | [0, 0, 0, 1]

These were computed symbolically using the transformation defined by DH convention and substituting values from the DH parameter table.

**Generalized homogeneous transform between base_link and gripper_link using end-effector pose:**

matrix row\column | 0 | 1 | 2 | 3
--- | --- | --- | --- | ---
0 | sin(p)cos(r)cos(y) + sin(r)sin(y) | -sin(p)sin(r)cos(y) + sin(y)cos(r) | cos(p)cos(y) | x
1 | sin(p)sin(y)cos(r) - sin(r)cos(y) | -sin(p)sin(r)sin(y) - cos(r)cos(y) | sin(y)cos(p) | y
2 | cos(p)cos(r) | -sin(r)cos(p) | -sin(p) | z
3 | 0 | 0 | 0 | 1

where r, p, y, x, y, and z are the roll, pitch, yaw, x-coordinate, y-coordinate, and z-coordinate of the end effector.

This transform was created by multiplying the following rotation matrices in order, then appending the position vector (x, y, z), then appending row 3 (0, 0, 0, 1) to complete the transform.
* rotation about the z-axis for `yaw` radians
* rotation about the y-axis for `pitch` radians
* rotation about the x-axis for `roll` radians
* rotation about the z-axis for pi radians
* rotation about the y-axis for -pi/2 radians

The final two rotations are to correct for the differences in the axis orientation of the gripper_link as defined by URDF vs DH convention. 
                                   
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Here is the diagram I used to derive the theta angles:

![alt text][image2]

First, the position of the wrist center is determined by subtracting the distance between the wrist center and the end-effector in the z-direction. Computing theta 1 is trivial when the wrist position is known. Theta 2 and 3 are solved by using the law of cosines to solve for the angles of the SSS triangle. Theta 5 is solved by plugging in known distances into the equation in the second red line on the right in the diagram above. Theta 4 and 6 are solved similarly, but the particular solution is chosen according to the sign of cos(theta5) to reduce the amount of rotation.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

[Here](https://github.com/dcato98/robotics-nd-p2/blob/master/kuka_arm/scripts/IK_server.py) is the completed `IK_server.py` file.

##### Discussion of Results, Performance, and Potential Areas for Improvement
The IK server now accurately computes the angles necessary for tracing the desired trajectory for the Kuka KR210 arm and successfully completes all pick-and-place cycles.

The IK server uses symbolic transforms, which simplify the calculations, but are themselves slow to compute and don't change from run-to-run. Computing the symbols in the VM on my laptop takes around 90 seconds. To prevent this redundant calculation, I chose to serialize the symbols using dill (a version of pickle) and deserialize them from disk when the file is available. Deserialization takes less than 1 second. I also added a global variable to cache the symbolic transforms for use by subsequent calls to the IK handler. Note that if both the cached symbols and the pickled symbols are not present, the symbols will be recalculated from scratch.

The bottleneck on IK server performance is now on calculating the angles, which takes roughly 0.1 seconds per joint trajectory point. This could be improved by doing the calculations in numpy which takes advantage of both parallelism and C++ code. While this is negligible compared to the performance bottleneck for the overall simulation (it takes 3-8 minutes to complete one pick-and-place cycle), a delay of a few of seconds is likely to be more observable in the real world where the no simulation is taking place.

A final area of improvement would be to reduce the amount of unnecessary wrist motion by selecting the IK solution that minimizes the distance travelled between trajectory points. A small reduction in unnecesary motion was accomplished by choosing angles for joints 4 and 6 according to the the sine of the joint 5 angle, but further improvements should be possible.

**Finally, just for fun, here's a picture of the robot in action:**

![alt text][image3]
