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
    Ti=trvec2tform(via_points(:,w)')*eul2tform(orientation(:,w)');
    Tf=trvec2tform(via_points(:,w+1)')*eul2tform(orientation(:,w+1)');
    via_time=via_point_time(:,w:w+1);
    traj_time=via_time(1):ts:via_time(2);
    
    traj_type='quintic';

    switch traj_type
        case 'trap'
            [s,sd,sdd] = trapveltraj([0 1],numel(traj_time), ... 
                'EndTime',diff(timeInterval));
        case 'cubic'
            [s,sd,sdd] = cubicpolytraj([0 1],via_time,traj_time);
        case 'quintic'
            [s,sd,sdd] = quinticpolytraj([0 1],via_time,traj_time);
        otherwise
               disp('Invalid Input');
    end
    
    [T,V,A]=transformtraj(Ti,Tf,via_time,traj_time,...
        'TimeScaling',[s;sd;sdd]);
     
    for idx=1:numel(traj_time)
        pos=T(:,:,idx);
        [config,sol]=ik('Gripper',pos,ikweights,ikinitguess);
        ikinitguess=config;
        q=tform2trvec(pos);
        if plot_type==1
            set(htraj,'xdata',[htraj.XData q(1)],...
                'ydata',[htraj.YData q(2)],...
                'zdata',[htraj.ZData q(3)]);
        elseif plot_type==2
            plotTransforms(tform2trvec(pos),tform2quat(pos),'FrameSize',0.09)
        end
        
        show(IRB6620_mdh,config,'Frames','off','PreservePlot',false);
        drawnow

    end
end
