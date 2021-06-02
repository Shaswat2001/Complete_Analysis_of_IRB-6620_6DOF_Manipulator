% Trajectory Generation in Task Space (NO ROTATION)

load IRB
Create_Via_Points
show(IRB6620_mdh,joint_position_home,'Frames','off',"PreservePlot",false);
% plot_type- 1 for trajectory Points : 2 for Coordinate Frames
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
orient.TargetOrientation=constraint_quat;
orient.OrientationTolerance = deg2rad(1);
ikinitguess=joint_position_home;
%% Trajectory Generation

traj_type='cubic';

switch traj_type
    case 'trapezoid'
        [q,qd,qdd]=trapveltraj(via_points,numel(traj_time),...
            'AccelTime',repmat(via_point_accl_time,[3 1]),'EndTime',repmat(diff(via_point_time),[3 1]));
    case 'cubic'
        [q,qd,qdd]=cubicpolytraj(via_points,via_point_time,traj_time,...
            'VelocityBoundaryCondition',via_point_vel);
    case 'quintic'
        [q,qd,qdd]=quinticpolytraj(via_points,via_point_time,traj_time,...
            'VelocityBoundaryCondition',via_point_vel,...
            'AccelerationBoundaryCondition',via_point_accl);
end

for idx=1:numel(traj_time)
    pos_tgt.TargetPosition=q(:,idx)';
    [config,sol]=gik(ikinitguess,pos_tgt,orient);
    ikinitguess=config;
    
    if plot_type==1
        set(htraj,'xdata',[htraj.XData q(1,idx)],...
            'ydata',[htraj.YData q(2,idx)],...
            'zdata',[htraj.ZData q(3,idx)]);
    elseif plot_type==2
        plotTransforms(q(:,idx)',repmat([1 0 0 0],[size(q(:,idx),2) 1]),'FrameSize',0.09);
    end
    
    show(IRB6620_mdh,config,'Frames','off','PreservePlot',false);
    xlabel("X[m]");
    ylabel("Y[m]");
    zlabel("Z[m]");
    title(['Time :' num2str(traj_time(idx)) 'sec']);
    drawnow
end
