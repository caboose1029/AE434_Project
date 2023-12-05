% Main - attempt 2

%% Given
Ix = 100;
Iy = 100;
Iz = 50;

w0_x = 0.2;
w0_y = 0.2;
w0_z = 1;
W0 = [w0_x w0_y w0_z]';

%% Calculate K1 and K2
A1 = [0   ((-Ix-Iz)/Iy); ((-Ix-Iz)/Iy) 0];
B1 = [1/Ix 0; 0 1/Iy];  
B2 = 1/Iz;

%  Feel free to adjust zeta and t_s
zeta = 0.75;
t_s = 60;
omega_n = 4 / (zeta * t_s);

real_pole = - zeta * omega_n;
imag_pole = omega_n * sqrt(1 - zeta^2);

poles = [real_pole+1j*imag_pole; real_pole-1j*imag_pole];

disp(poles)

K1 = place(A1,B1,poles);

tau = t_s / 4;

K2 = 1/(B2*tau);

%% Setup for diffEqs
W01 = [w0_x w0_y]';
W02 = w0_z;
I = diag([Ix Iy Iz]);

% Define timespan for integration - change multiplier as desired
T = t_s * 4;
tspan = linspace(0,T,1000);

% Integration options
tol = 1e-13;
options = odeset('RelTol', tol, 'AbsTol', tol);

%% Integrations for velocity controllers
[txy,xy] = ode45(@(t, xy) diffEq(t, xy, K1, I), tspan, W01, options);

% Find settling time for transverse velocity controller
threshold = 0.02 * norm(W0(1:2));
xy_norm = vecnorm(xy, 2,2);
ts_index = find(xy_norm < threshold, 1);
ts = txy(ts_index);

% Define time span for axial velocity integration
tz_span = linspace(ts, T, 1000);

% Integration for axial velocity controller
[tz, z] = ode45(@(t, z) diffEq_z(t, z, K2, I), tz_span, W02, options);

%% Setup for plotting
% Combine results for graphical representation
t_combined = [txy(1:ts_index); tz];
xy_combined = [xy(1:ts_index, :); repmat(xy(ts_index, :), length(tz), 1)];
z_combined = [W02 * ones(ts_index, 1); z];

% Plot the angular velocities over time
plot2d(t_combined,xy_combined,z_combined)


%% Functions
function dxdt = diffEq(t, W0, K, I)
% Integrates transverse velocity proportional gain controller

x = W0;

u = -K * x;

I_x = I(1,1);
I_y = I(2,2);
I_z = I(3,3);


    A = [0   ((-I_x-I_z)/I_y); ((-I_x-I_z)/I_y) 0];
    B = [1/I_x 0; 0 1/I_y];  

    dxdt = A*x + B*u;

end

function dxdt = diffEq_z(t, W0, K2, I)
% Integrates axial velocity proportional gain controller

x = W0;

r = 0;
u = K2 * (r - x);

I_z = I(3,3);
  
    B = 1/I_z;

    dxdt = B * u;

end

function plot2d(t, xy, z)

% Define angular velocity matrices for Wx, Wy, and Wz
Wx = xy(:,1);
Wy = xy(:,2);
Wz = z(:);
    
    figure(1)
    hold on
    
    plot(t,Wx,'g');
    plot(t,Wy,'b');
    plot(t,Wz,'r');
    title('Angular velocity components over time (numerical)' );
    
    xlabel('Time(s)');
    ylabel('Angular velocity (degrees/sec)');
    legend('Wx','Wy','Wz');
    
    hold off
end