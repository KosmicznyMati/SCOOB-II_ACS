
% Actuators Function

function [H_rw_new,omega_rw_new,omega_dot_rw,omega_rw_new_RPM,V_rw_new,T_available,desaturation_mode,sat_dir] = actuators(J_RW,rw_align,v_V_conversion,omega_rw,T_control,dt,m,B,desaturation_mode,omega_rw_saturated,omega_rw_desaturated,sat_dir)

    if desaturation_mode == 0
%         [H_rw_new,omega_rw_new,omega_dot_rw,omega_rw_new_RPM,V_rw_new] = rw(J_RW,rw_align,v_V_conversion,omega_rw,T_control,dt,desaturation_mode);
        omega_dot_rw = omega_rw_dot(J_RW,T_control);
        omega_rw_new = omega_rw + omega_dot_rw*dt;    % First-order numerical integration

        H_rw_new = rw_align*J_RW*omega_rw_new;        % Angular momentum of reaction wheels matrix

        %Unit Conversion
        omega_rw_new_RPM = omega_rw_new/(2*pi);
        V_rw_new = omega_rw_new/v_V_conversion;
        
        sat_dir =[0; 0; 0];
        
        if abs(omega_rw_new) >= omega_rw_saturated
            desaturation_mode = 1;
            count = 1;
            for omega = omega_rw_new
                if omega > omega_sat_p
                    sat_dir(count) = 1;
                elseif omega < omega_sat_n
                    sat_dir(count) = -1;
                else
                    sat_dir(count) = 0;
                end
                count = count + 1;
            end
        end
    end
    
    if desaturation_mode == 1
        % Check which reaction wheel is saturated and in which direction
        
        T_available = mtq(sat_dir.*m,B);                % Calculate Torque magnetorquers can provide in the direction of current spinning of RWs
        omega_dot_rw = omega_rw_dot(J_RW,T_available);  % Calculate rate of change of omega based on the torque torquers can provide MAKE SURE IT'S IN THE OPPOSITE DIRECTION TO THE CURRENT ROTATION FOR THE WHEEL WE WANT TO DESATURATE
        
        omega_rw_new = omega_rw + (-1)*omega_dot_rw*dt; % Omega_dot *(-1), as RWs are to decelerate wheels (provide torque in the opposite direction than the torquers)
        
%         mtq_status = 1;                     % Turn on only magnetorquers respective to the wheels we're desaturating.
      
        if abs(omega_rw_new) <= omega_rw_desaturated
            desaturation_mode = 0;
        end
    end
end

% Reaction Wheel model

function [H_rw_new,omega_rw_new,omega_rw_new_RPM,V_rw_new] = rw(J_RW,rw_align,rw_conversion,omega_rw,T_control,dt)
  
    omega_dot_rw = omega_rw_dot(J_RW,T_control);
    omega_rw_new = omega_rw + omega_dot_rw*dt;    % First-order numerical integration
  
    H_rw_new = rw_align*J_RW*omega_rw_new;                        % Angular momentum of reaction wheels matrix
  
    %Unit Conversion
    omega_rw_new_RPM = omega_rw_new/(2*pi);
    V_rw_new = omega_rw_new/rw_conversion;
end

function omega_rw_dot = omega_rw_dot(J_RW,T_control)
    omega_rw_dot = J_RW\T_control;
%     if omega_rw_dot >= 100
%         omega_rw_dot = 100;
%     elseif omega_rw_dot <= -100
%         omega_rw_dot = -100;
%     end
end

% Magnetorquer model

function T_available = mtq(m,B)
    T_available = Cross3x33(m)*B;
end

% function V_mtq = mtq(R_wire,L_wire,r,mu_r,N_d,B,T_control)
%     M_dipole = -Cross3x33(B)\T_control;
%     const = 1 + (mu_r - 1)/(1 + N_d*(mu_r - 1));
%     V_mtq = M_dipole*(2*R_wire)/(const*r*L_wire);
% end

% 3x3 Cross Product Operator for a 3x1 Vector

function cross = Cross3x33(v)
cross = [0 -v(3) v(2); 
         v(3) 0 -v(1); 
        -v(2) v(1) 0]; 
end