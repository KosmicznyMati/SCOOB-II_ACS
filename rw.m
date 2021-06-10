
% Reaction Wheel model (PERFORM MOTOR CHARACTERISATION, TRY SENDING
% COMMANDS FROM MATLAB START UP TIME, RISE TIME, CONNECT REACTION WHEELS)

function [H_rw_new,omega_rw_new,omega_dot_rw,omega_rw_new_RPM,V_rw_new] = rw(J_RW,rw_align,rw_conversion,omega_rw,T_control,dt)
    
    omega_dot_rw = omega_rw_dot(J_RW,T_control);
    omega_rw_new = omega_rw + omega_dot_rw*dt;    % First-order numerical integration
  
    H_rw_new = rw_align*J_RW*omega_rw_new;                        % Angular momentum of reaction wheels matrix
  
    %Unit Conversion
    omega_rw_new_RPM = omega_rw_new/(2*pi);
    V_rw_new = omega_rw_new/rw_conversion;
end

function omega_rw_dot = omega_rw_dot(J_RW,T_control)
omega_rw_dot = J_RW\T_control;
end