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