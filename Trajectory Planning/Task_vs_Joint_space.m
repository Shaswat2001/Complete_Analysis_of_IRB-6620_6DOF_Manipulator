% Compares the Task space and joint space trajectory

%% Setup

load IRB
% Define via-point information
Create_Via_Points
% Plotting the via-points 
hold on
grid on
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro--');
xlabel("X[m]");
ylabel("Y[m]");
zlabel("Z[m]");
title("Via Points");
view([45 45]);

% Creating Inverse Kinematics Solver
% Generalized Inverse Kinematics is chosen to add orientation constraints
% to the gripper
gik=generalizedInverseKinematics('RigidBodyTree',IRB6620_mdh,...
    'ConstraintInputs',{'position','orientation'});
% Creating position constraint object
postgt=constraintPositionTarget('Gripper');
% Creating orientation constraint object
orient=constraintOrientationTarget('Gripper');
% Orientation of Gripper is set constant
orient.TargetOrientation=constraint_quat;
orient.OrientationTolerance = deg2rad(1);

joint_task=zeros(num_joints,numel(traj_time));
pos_task=zeros(3,numel(traj_time));
%% Trajectory Generation in Task Space
% Starting timer
tic

% Solving Trapezoidal Trajectory in Task Space
[q,qd,qdd]=trapveltraj(via_points,numel(traj_time),...
    'AccelTime',repmat(via_point_accl_time,[3 1]),'EndTime',repmat(diff(via_point_time),[3 1]));

for idx=1:numel(traj_time)
    %Solving Inverse Kinematics at different Gripper Positions
    postgt.TargetPosition=q(:,idx)';
    [config,sol]=gik(ikinitguess,postgt,orient);
    joint_task(:,idx)=config';
    ikinitguess=config;    
end

Task_time=toc;
disp(['Time Taken to Solve in Task Space : ' num2str(Task_time) 'secs']);

%% Trajectory Generation in Joint Space

num_via_points=size(via_points,2);
joints_val=zeros(num_joints,num_via_points);
% Setting the initial position of robot
ikinitguess=joint_position_home;

tic

for idx=1:num_via_points
    % Finding the configuration of robot at via points
    postgt.TargetPosition=via_points(:,idx)';
    [config,sol]=gik(ikinitguess,postgt,orient);
    joints_val(:,idx)=config';
    ikinitguess=config;
    
end

% Solving Trapezoidal Trajectory in Joint Space
[jq,jqd,jqdd]=trapveltraj(joints_val,numel(traj_time),...
    'AccelTime',repmat(via_point_accl_time,[num_joints 1]),...
    'EndTime',repmat(diff(via_point_time),[num_joints 1]));

for idx=1:numel(traj_time)
    % End Effector position during the trajectory
    T=getTransform(IRB6620_mdh,jq(:,idx)','Gripper');
    pos_task(:,idx)=tform2trvec(T);
end

joint_time=toc;
disp(['Time Taken to Solve in Joint Space : ' num2str(joint_time) 'secs']);

%% Plotting Both the trajectories

close all
figure
hold on
grid on
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro');
plot3(q(1,:),q(2,:),q(3,:),'r-');
plot3(pos_task(1,:),pos_task(2,:),pos_task(3,:),'b--');
xlabel("X[m]");
ylabel("Y[m]");
zlabel("Z[m]");
title("Task vs Joint Space Trajectory");
legend('Via Points','Task Space Trajectory','Joint Space Trajectory');

% Comparing Joint Angles in Task and Joint Space Trajectory
for idx=1:num_joints
    figure
    hold on
    plot(traj_time,jq(idx,:)');
    plot(traj_time,joint_task(idx,:));
    xlabel("Time [sec]");
    ylabel("Joint Angle [rad]");
    title(['Joint ' num2str(idx) ' Trajectory']);
    legend('Joint Space','Task Space');
end