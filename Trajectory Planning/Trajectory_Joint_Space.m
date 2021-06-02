% Trajectory Generation in Joint Space (NO ROTATION)

load IRB
Create_Via_Points
show(IRB6620_mdh,joint_position_home,'Frames','off','PreservePlot',false);
% plot_type - 1 for trajectory Points : 2 for Coordinate Frames
plot_type=1;
hold on
grid on
if plot_type==1
    htraj=plot3(via_points(1,1),via_points(2,1),via_points(3,1),'b.-');
end
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro--');

%% Inverse Kinematic Solver

gik=generalizedInverseKinematics('RigidBodyTree',IRB6620_mdh,...
    'ConstraintInputs',{'position','orientation'});

pos_tgt=constraintPositionTarget('Gripper');

orient=constraintOrientationTarget('Gripper');
% Orientation of Gripper is set constant
orient.TargetOrientation=constraint_quat;
orient.OrientationTolerance = deg2rad(1);

%% Trajectory in Joint Space
ikinitguess=joint_position_home;
num_via_points=size(via_points,2);
joint_val=zeros(num_joints,num_via_points);

for idx=1:num_via_points
    pos_tgt.TargetPosition=via_points(:,idx)';
    [config,sol]=gik(ikinitguess,pos_tgt,orient);
    ikinitguess=config;
    joint_val(:,idx)=config';
end

traj_type='trapezoid';

switch traj_type
    
    case 'trapezoid'
        [q,qd,qdd]=trapveltraj(joint_val,numel(traj_time),...
            'AccelTime',repmat(via_point_accl_time,[num_joints 1]),...
            'EndTime',repmat(diff(via_point_time),[num_joints 1]));
    case 'cubic'
        [q,qd,qdd]=cubicpolytraj(joint_val,via_point_time,traj_time,...
            'VelocityBoundaryCondition',zeros(num_joints,num_via_points));
    case 'quintic'
        [q,qd,qdd]=quinticpolytraj(joint_val,via_point_time,traj_time,...
            'VelocityBoundaryCondition',zeros(num_joints,num_via_points),...
            'AccelerationBoundaryCondition',zeros(num_joints,num_via_points));
        
end

%% PLotting the Result

for idx=1:numel(traj_time)
    T=getTransform(IRB6620_mdh,q(:,idx)','Gripper');
    pos=tform2trvec(T);
    if plot_type==1
        set(htraj,'xdata',[htraj.XData pos(1)],...
            'ydata',[htraj.YData pos(2)],...
            'zdata',[htraj.ZData pos(3)]);
    elseif plot_type==2
        plotTransforms(tform2trvec(T),tform2quat(T),'FrameSize',0.09);
    end
    
    show(IRB6620_mdh,q(:,idx)','Frames','off','PreservePlot',false);
    title(['Time : ' num2str(traj_time(idx)) 'secs']);
    xlabel("X[m]");
    ylabel("Y[m]");
    zlabel("Z[m]");
    drawnow
end