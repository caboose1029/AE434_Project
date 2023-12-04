function dxdt = diffEq_z(t, W0, K2, I)
% Integrates axial velocity proportional gain controller

x = W0;

r = 0;
u = K2 * (r - x);

I_z = I(3,3);
  
    B = 1/I_z;

    dxdt = B * u;

end