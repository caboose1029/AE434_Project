classdef Spacecraft < handle
    % Class to describe spacecraft in DCM and quaternion representations
    %
    %

    properties
        Inertia_Tensor
        omega_0
        Orientation
        Sim_Time
    end
    
    methods
        function obj = Spacecraft(I, w_0, eDCMb, T)
            obj.Inertia_Tensor = I;
            obj.omega_0 = w_0;
            obj.Orientation = eDCMb;
            obj.Sim_Time = T;

            
        end
        
        function updatePosition(obj)
            T = obj.Sim_Time;
            tspan = linspace(0,T,1000);
            tol = 1e-13;
            I = obj.Inertia_Tensor;
            W0 = obj.omega_0;
            %Q0 = obj.quaternion_0;
            y0 = [W0];

            options = odeset('RelTol',tol, 'AbsTol',tol);

            [t, x] = ode45(@diffEq, tspan, y0, I, options);
        end

    end

   

end