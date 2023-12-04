function plot2d(t, xy, z)

% Define angular velocity matrices for Wx, Wy, and Wz
Wx = xy(:,1);
Wy = xy(:,2);
Wz = z(:);
    
    figure(1)
    hold on
    
    plot(t,Wx,'g');
    plot(t,Wy-0.01*Wy,'b');
    plot(t,Wz,'r');
    title('Angular velocity components over time (numerical)' );
    
    xlabel('Time(s)');
    ylabel('Angular velocity (degrees/sec)');
    legend('Wx','Wy','Wz');
    
    hold off

    % figure(2)
    % hold on
    % 
    % plot(t,x(:,4),'g');
    % plot(t,x(:,5),'b');
    % plot(t,x(:,6),'r');
    % plot(t,x(:,7));
    % title('Quaternion components over time (numerical)')
    % 
    % xlabel('Time(s)');
    % ylabel('Quaternion');
    % legend('q_1','q_2','q_3','q_4');
    % 
    % hold off
end