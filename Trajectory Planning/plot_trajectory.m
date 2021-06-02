function [] = plot_trajectory(q,qd,qdd,type,time)
    num_plt=size(q,1);
    cart=['X' 'Y' 'Z'];
    for i=1:num_plt
        figure
        
        
        
        subplot(3,1,1)
        plot(time,q(i,:))
        xlabel('Time [sec]');
        ylabel('Position');
        
        if type=="cart_space"
            title(['Trajectory in ' cart(i) ' Coordinate']);            
        elseif type=="joint_space"
            title(['Trajectory of Joint ' num2str(i)]);
        end
        subplot(3,1,2)
        plot(time,qd(i,:))
        xlabel('Time [sec]');
        ylabel('Velocity');
        
        subplot(3,1,3)
        plot(time,qdd(i,:))
        xlabel('Time [sec]');
        ylabel('Acceleration');
    end
end