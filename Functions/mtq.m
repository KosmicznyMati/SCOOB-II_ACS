
% Magnetorquer model

function V_mtq = mtq(R_wire,L_wire,r,mu_r,N_d,B,T_control)
    M_dipole = -Cross3x33(B)\T_control;
    const = 1 + (mu_r - 1)/(1 + N_d*(mu_r - 1));
    V_mtq = M_dipole*(2*R_wire)/(const*r*L_wire);
end

% 3x3 Cross Product Operator for a 3x1 Vector

function cross = Cross3x33(v)
cross = [0 -v(3) v(2); 
         v(3) 0 -v(1); 
        -v(2) v(1) 0]; 
end