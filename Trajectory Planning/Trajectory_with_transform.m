% Trajectory Generation in Task Space (Both TRANSLATION and ROTATION)

load IRB
% Define via-point information
Create_Via_Points
plot_type=2;
show(IRB6620_mdh,joint_position_home);
hold on
grid on
% plot_type - 0 = None,1 = Trajectory, 2 = Coordinate Frames
if plot_type==1
    htraj=plot3(via_points(1,1),via_points(2,1),via_points(3,1),'b.-');
end
plot3(via_points(1,:),via_points(2,:),via_points(3,:),'ro--');

% Inverse Kinematic Solver
ik=inverseKinematics('RigidBodyTree',IRB6620_mdh);
ikinitguess=IRB6620_mdh.homeConfiguration;
%% Trajectory Generation 

num_via_points=size(via_points,2);

for w=1:num_via_points-1
    % Create Trajectory between consecutive via-points
    Ti=trvec2tform(via_points(:,w)')*eul2tform(orientation(:,w)');
    Tf=trvec2tform(via_points(:,w+1)')*eul2tform(orientation(:,w+1)');
    % time at via points
    via_time=via_point_time(:,w:w+1);
    % trajectory time
    traj_time=via_time(1):ts:via_time(2);
    
    % Time scaling 
    traj_type='cubic';
    switch traj_type
        case 'trap'
            [s,sd,sdd] = trapveltraj([0 1],numel(traj_time), ... 
                'EndTime',diff(timeInterval));
        case 'cubic'
            [s,sd,sdd] = cubicpolytraj([0 1],via_time,traj_time);
        case 'quintic'
            [s,sd,sdd] = quinticpolytraj([0 1],via_time,traj_time);
        otherwise
               error("Invalid trajectory type! Use ''trapezoid'', ''cubic'' or ''quintic'' ");
    end
    % Transforms obtained from trajectory generation
    [T,V,A]=transformtraj(Ti,Tf,via_time,traj_time,...
        'TimeScaling',[s;sd;sdd]);
     
    for idx=1:numel(traj_time)
        % Performing inverse kinematic analysis
        pos=T(:,:,idx);
        [config,sol]=ik('Gripper',pos,ikweights,ikinitguess);
        ikinitguess=config;
        q=tform2trvec(pos);
        % Ploting the result
        if plot_type==1
            set(htraj,'xdata',[htraj.XData q(1)],...
                'ydata',[htraj.YData q(2)],...
                'zdata',[htraj.ZData q(3)]);
        elseif plot_type==2
            plotTransforms(tform2trvec(pos),tform2quat(pos),'FrameSize',0.09)
        end
        
        % Visualize the robot
        show(IRB6620_mdh,config,'Frames','off','PreservePlot',false);
        xlabel("X[m]");
        ylabel("Y[m]");
        zlabel("Z[m]");
        drawnow
    end
end