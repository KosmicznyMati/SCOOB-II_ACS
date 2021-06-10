
% Fourth Order Runge-Kutta

function [omega_new,q_new,omega_dot_new] = RK4(J,J_inv,H,q,omega,dt,T_c)

  k1 = omega_dot(J,J_inv,H,omega,T_c);              % Approx for y gives approx for deriv
  k2 = omega_dot(J,J_inv,H,omega + k1*dt/2,T_c);    % Approx deriv at intermediate value.
  k3 = omega_dot(J,J_inv,H,omega + k2*dt/2,T_c);    % Another approx deriv at intermediate value.
  k4 = omega_dot(J,J_inv,H,omega + k3*dt,T_c);      % Approx deriv at endpoint value.
  omega_new = omega + (k1+2*k2+2*k3+k4)*dt/6;       % Approx soln 
  omega_dot_new = omega_dot(J,J_inv,H,omega_new,T_c);% Taken the new omega dot to be the omega dot of the new angular velocity
  
  k1 = Q_Dot(q,omega);
  k2 = Q_Dot(q + k1*dt/2,omega);
  k3 = Q_Dot(q + k2*dt/2,omega);
  k4 = Q_Dot(q + k3*dt,omega);
  q_new = q + (k1+2*k2+2*k3+k4)*dt/6;

end

% Rate of Change of Omega

function omega_dot = omega_dot(J,J_inv,H,omega,T)
omega_dot = J_inv*((-Cross3x33(omega)*(J*omega+H)) + T);  % From Yang
end

% Rate of Change of Quaternion

function q_dot = Q_Dot(q,omega)             % 4x1 quaternion input, 3x1 angular velocity input
q_dot = 0.5*Cross4x43(omega)*q;             % Taken from Yang
end

% 3x3 Cross Product Operator for a 3x1 Vector

function cross = Cross3x33(v)
cross = [0 -v(3) v(2); 
         v(3) 0 -v(1); 
        -v(2) v(1) 0]; 
end

% 4x4 Cross Product Operator for a 3x1 Vector

function cross = Cross4x43(v)
cross = [0 -v(1) -v(2) -v(3); 
        v(1) 0    v(3) -v(2); 
        v(2) -v(3) 0    v(1); 
        v(3)  v(2) -v(1) 0]; 
end