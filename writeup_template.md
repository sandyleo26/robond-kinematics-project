## Project: Kinematics Pick & Place
---
In this project, I will be writing code to perform Inverse Kinematics, meaning given a list of end-effector poses, you will calculate joint angles for the Kuka KR210.

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[wc-equation]: ./misc_images/wc-equation.png
[theta123]: ./misc_images/theta123.png

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### 1. Writeup / README
> Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### 2. Kinematic Analysis
> Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

`kr210.urdf.xacro` contains all the robot specific information like links, joints, actuators, etc. Take the following snippet for example, we can conclude that `joint_1` is a revolute that rotates about `z` axis and link `base_link` and `link_1`.


```
<joint name="joint_1" type="revolute">
  <origin xyz="0 0 0.33" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="link_1"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
```

Same as above we can get the following info from it. 

- J0 = (0, 0, 0)
- J1 = (0, 0, 0.33)
- J2 = (0.35, 0, 0.42)
- J3 = (0, 0, 1.25)
- J4 = (0.96, 0, -0.054)
- J5 = (0.54, 0, 0)
- J6 = (0.193, 0, 0)
- JG = (0.11, 0, 0)

Next, using those info, DH tables can be constructed as below. 

Links | alpha(i-1) | a(i-1) | d(i-1) | q(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2.0 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2.0 | -0.054 | 1.5 | q4
4->5 | pi/2.0 | 0 | 0 | q5
5->6 | -pi/2.0 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

- Twist angle (alpha) is the angle between `z_i-1` and `z_i` as measured about `x_i-1` in the right-hand sense
- Link length (a) is the distance between `z_i-1` and `z_i` along `x_i-1` where `x_i-1` is perpendicular to both `z_i-1` and `zi`
- Link offset (d) is the signed distance from `x_i-1` to `x_i` measure along `z_i`. Will be a variable for a prismatic joint.
- Joint angle (q) is the angle between `x_i-1` to `x_i` measured about `z_i` in the right-hand send. Will be a variable for a revolute joint.


> Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I define a homogeneous transform helper function for the DH coordinates system from joint `i-1` to `i` is below, where `alpha`, `a`, `d` and `q` are define as above.

    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([
                    [           cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                0,                 0,           0,             1]])
            return TF

Using this helper function, each individual transformation matrix can be define as:

        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

And the overall TF matrix is all above post-multiplied:

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First, the **inverse position** problem. Using the equation below,

![wc equation][wc-equation]

We can get write center position as

	WC = EE - d * ROT_EE[:,2]
	
Where 

* `EE` is end-effector
* `d` is distance between WC to EE
* `ROT_EE` is corrected complete rotation matrix.

Then using the following visual, we can calculate `theta1`, `theta2` and `theta3` using law of cosine and SSS trigonometry.

![theta1, theta2, theta3][theta123]

    theta1 = atan2(WC[1], WC[0])
    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] ** 2 + WC[1] ** 2) - 0.35)
    theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

Second, for the **inverse orientation** problem,

We know that

	R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6 = ROT_EE

And

	R3_6 = inv(R0_3) * ROT_EE

And using `atan2` we get

	theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	theta5 = atan2(sqrt(R3_6[0,2] ** 2 + R3_6[2,2] ** 2), R3_6[1,2])
	theta6 = atan2(-R3_6[1,1], R3_6[1,0])


### 4. Project Implementation

> Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

First, I define symbos, DH tables, and individual transformation matrices out side the loop.

     # define DH parameter symbols
     q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta
     d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
	...
	DH_Table = {
	
Then in the loop, 	    

	# define roll, pitch, yaw rotation matrices
	r, p, y = symbols('r p y')
	ROT_x = Matrix([
		[1, 0, 0],
		[0, cos(r), -sin(r)],
		[0, sin(r), cos(r)]])
	...
	
And calculate `theta1` ~ `theta6` using equations listed above.

### 5. Challenges in the future
* Using numpy to speed up

