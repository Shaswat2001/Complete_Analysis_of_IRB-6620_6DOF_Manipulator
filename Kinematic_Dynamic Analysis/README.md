# Kinematic_Dynamic_Analysis_of_IRB6620

Kinematic and Dynamic analysis is the foundation of any robot. To perform the analysis detailed analytical approach validated with a numerical software approach of IRB 6620 is performed. The robotics Toolbox in MATLAB is used to create IRB 6620 Manipulator tree (IRB_6620_Model.m). The Manipulator model in MATLAB is is shown in the Fig.  

To create IRB 6620 using the Toolbox, the DH parameters from Table are used as inputs.

Moving on to the kinematic and Dynamic analysis.

## Kinematics

It is the science of motion that treats the subject without regard to the forces that cause it. There are two types of problems in kinematics. First one is Inverse Kinematics where given end effector pose, joint angles are calculated. Other is known as Forward Kinematics, where given the joint angles, the end effector orientation and position is calculated. 

### Forward Kinematics

We start with analytical equation of forward kinematics first as shown. 

To get the homogenous transformation matrix of frame relative to another frame in MATLAB getTransform() function is used. 

The homogenous matrix obtained by this software numerical approach yields the same result as that of the analytical approach. 

### Inverse Kinematics

For IRB 6620, both analytical as well as simulation software approaches are used. The algebraic solution of IRB 6620 is presented first. 

To perform Inverse Kinematics in MATLAB two functions can be used, inverseKinematics() and generalizedInverseKinematics(). generalizedInverseKinematics() is used when there are more constraints apart from pose of the end effector. The results obtained is shown in Fig.

## Static Velocity Analysis

Since manipulator is a chain of bodies, each one is capable of motion relative to its neighbor. Velocity of link i+1 will be addition of velocity of link i and velocity component added by joint i+1. While determining the motion of robot links, link frame 0 is taken as reference. For simplicity in the calculation, linear and angular velocity are calculated at [0 pi/2 0 0 0 0] joint values. 
