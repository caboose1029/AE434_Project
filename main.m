% Main Program

% Given initial conditions
I = diag([100 100 50]);     % Inertia tensor
W0 = [0.2 0.2 1]';           % Initial angular velocity [rad/sec]
Euler_0 = [0 pi/2 0];       % Initial orientation [phi_0 theta_0 psi_0]

% Calculate DCM and Quaternions for initial orientation
% eDCMb = angle2dcm(Euler_0(1), Euler_0(2), Euler_0(3), "ZXZ");
% Q = eul2quat(Euler_0, 'ZXZ');

% Seperate angular velocities into xy and z components
W01 = W0(1:2);
W02 = W0(3);

% Define gain for angular velocity control
K1 = [200 0; 0 200];
K2 = 200;

% Define timespan for integration
T = 5;
tspan = linspace(0,T,1000);

% Integration options
tol = 1e-13;
options = odeset('RelTol', tol, 'AbsTol', tol);

% Integration for transverse velocity controller
[txy,xy] = ode45(@(t, xy) diffEq(t, xy, K1, I), tspan, W01, options);

% Find settling time for transverse velocity controller
threshold = 0.02 * norm(W0(1:2));
xy_norm = vecnorm(xy, 2,2);
ts_index = find(xy_norm < threshold, 1);
t_s = txy(ts_index);

% Define time span for axial velocity integration
tz_span = linspace(t_s, T, 1000);

% Integration for axial velocity controller
[tz, z] = ode45(@(t, z) diffEq_z(t, z, K2, I), tz_span, W02, options);

% Combine results for graphical representation
t_combined = [txy(1:ts_index); tz];
xy_combined = [xy(1:ts_index, :); repmat(xy(ts_index, :), length(tz), 1)];
z_combined = [W02 * ones(ts_index, 1); z];

% Plot the angular velocities over time
plot2d(t_combined,xy_combined,z_combined)