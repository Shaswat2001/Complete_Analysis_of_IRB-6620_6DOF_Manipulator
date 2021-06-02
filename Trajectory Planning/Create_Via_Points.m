% Create Sample Data points for Trajectory Generation

load IRB
load IRBPosition
% End effector name
eename='Gripper';
ikweights=[1 1 1 1 1 1];
ikinitguess=joint_position_home;

% Array of Via point time
via_point_time=0:4:16;

% Quaternion for constant orientation of Gripper
constraint_quat=[0 -0.7071 0 -0.7071];

% num of joints in the robot
num_joints=numel(IRB6620_mdh.homeConfiguration);
% Position (X Y Z)
via_points=tool_position_home'...
    +[0 0 0; -0.9 0.5 0.3 ; -0.4 -0.1 0.5 ;  -0.8 -0.5 0.4 ; 0 0 0]';

ts=0.2;
traj_time=0:ts:via_point_time(end);
% Acceleration time for Trapezoidal Trajectory
via_point_accl_time=diff(via_point_time)/4;

% orientation (Euler Angles) wrt home orientation
orientation=[0     0    0;
             pi/4  0    0; 
             0    pi/8  0;
            -pi/4  pi/4    0;
             0     pi/8    0]'; 
         
% Velocity Constraints for Cubic and Quintic Trajectory         
via_point_vel = 0.1 *[ 0  1  0;
                     -1  0  0;
                      0 -1  0;
                      1  0  0;
                      0  1  0]';
                  
% Acceleration Constraints Quintic Trajectory
via_point_accl = zeros(size(via_point_vel));
