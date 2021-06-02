load IRB
Create_Via_Points
plot_type=2;
show(IRB6620_mdh,joint_position_home);
hold on
grid on
if plot_type==1
    htraj=plot3(via_points(1,1),via_points(2,1),via_points(3,1),'b.-');
end
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro--');

ik=inverseKinematics('RigidBodyTree',IRB6620_mdh);


num_via_points=size(via_points,2);
for w=1:num_via_points-1
    Ri=eul2quat(orientation(:,w)');
    Rf=eul2quat(orientation(:,w+1)');
    via_time=via_point_time(:,w:w+1);
    traj_time=via_time(1):ts:via_time(2);
    
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
               disp(['Invalid Input']);
            

    end
    
    for idx=1:numel(traj_time)
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
        
        show(IRB6620_mdh,config,'Frames','off','PreservePlot',false);
        drawnow

    end
end
