% Task Space trajectory with linear interpolation in ROTATION

load IRB
% Define via-point information
Create_Via_Points
plot_type=2;
show(IRB6620_mdh,joint_position_home,'Frames','off','PreservePlot',false);
% plot_type - 0 = None,1 = Trajectory, 2 = Coordinate Frames
hold on
grid on
if plot_type==1
    htraj=plot3(via_points(1,1),via_points(2,1),via_points(3,1),'b.-');
end
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro--');

% IK Solver
ik=inverseKinematics('RigidBodyTree',IRB6620_mdh);
% Number of via points
num_via_points=size(via_points,2);
for w=1:num_via_points-1
    % Get the initial and final rotations and times for the segment
    Ri=eul2quat(orientation(:,w)');
    Rf=eul2quat(orientation(:,w+1)');
    via_time=via_point_time(:,w:w+1);
    traj_time=via_time(1):ts:via_time(2);
    
    % Find the quaternions from trajectory generation
    [R,omega,alpha]=rottraj(Ri,Rf,via_time,traj_time);
    
    traj_type='quintic';

    switch traj_type
        case 'trapezoid'
            [q,qd,qdd]=trapveltraj(via_points(:,w:w+1),numel(traj_time),...
                'AccelTime',repmat(diff(via_time)/4,[3 1]),'EndTime',repmat(diff(via_time),[3 1]));
            
        case 'cubic'
            [q,qd,qdd]=cubicpolytraj(via_points(:,w:w+1),via_time,traj_time,...
                'VelocityBoundaryCondition',via_point_vel(:,w:w+1));
            
        case 'quintic'
            [q,qd,qdd]=quinticpolytraj(via_points(:,w:w+1),via_time,traj_time,...
                'VelocityBoundaryCondition',via_point_vel(:,w:w+1),...
                'AccelerationBoundaryCondition',via_point_accl(:,w:w+1));
            
        otherwise
               error("Invalid trajectory type! Use ''trapezoid'', ''cubic'' or ''quintic'' ");
    end
    
    for idx=1:numel(traj_time)
        % solve ik
        pos=trvec2tform(q(:,idx)')*quat2tform(R(:,idx)');
        [config,sol]=ik('Gripper',pos,ikweights,ikinitguess);
        ikinitguess=config;
        
        if plot_type==1
            set(htraj,'xdata',[htraj.XData q(1,idx)],...
                'ydata',[htraj.YData q(2,idx)],...
                'zdata',[htraj.ZData q(3,idx)]);
        elseif plot_type==2
            plotTransforms(q(:,idx)',R(:,idx)','FrameSize',0.09)
        end
        % visualize the robot
        show(IRB6620_mdh,config,'Frames','off','PreservePlot',false);
        drawnow

    end
end
