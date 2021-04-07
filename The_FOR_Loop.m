

% Definitions & Starting Parameters

% Time
n  = 10000;   % number of time steps
dt = 0.01;    % time step in seconds

% Initialising arrays
q_array = zeros(n+1,4);
omega_array = zeros(n+1,3);
q_d_array = zeros(n+1,4);

q_dot_array = zeros(n+1,4);
omega_dot_array = zeros(n+1,3);

q_error_array = zeros(n+1,4);
omega_error_array = zeros(n+1,3);

omega_rw_array = zeros(n+1,3);

% Inputing initial conditions into the arrays
q_array(1,:) = q_initial(:);
omega_array(1,:) = omega_initial(:);
omega_rw_array(1,:) = omega_rw_initial(:);
q_d_array(1,:) = q_d(:);

% The Loop

for count = 1:n
    % Extracting the current orientation parameters from the array
    q = q_array(count,:)';          % Extracting the current attitude quaternion
    omega = omega_array(count,:)';  % Extracting the current omega
    omega_rw= omega_rw_array(count,:)';
    q_d = q_d_array(count,:)';      % Extracting current desired quaternion
    omega_dot = omega_dot_array(count,:)';
    
    % Normalising the current quaternion (move out of for loop? or leave for redundancy?
    q = Q_Normalise(q);
    
    % Error computations
%     q_error = Q_Error(q,q_d);                   % Attitude error quaternion (calculations based on Oland PD+ paper)
    omega_error = omega_d - omega;              % Omega error
    omega_dot_error = omega_dot_d - omega_dot;  % Omega dot error
    
    % The control law PD (based on Sarthak's pic, it works better, don't know why)
%     T = K_p*q_error(2:end)*q_error(1) + K_d*omega_error;  % Control torque vector
    % Alt PD control law for non-zero desired omega
    T = K_p*omega_error + K_d*omega_dot_error;
    
    % Decomposition into RWs
    [H,omega_rw_new] = reaction_wheels(J_RW,rw_align,omega_rw,T,dt);

    % Runge-Kutta Fourth Order numerical integration, output q normalisation
    [omega_new,q_new,omega_dot_new] = RK4(J,J_inv,H,q,omega,dt,T);
    q_new = Q_Normalise(q_new);
    
    % Changing desired quaternion
%     q_d_new = q_d_change(q_d,q_dot_d,dt);

    % Storing the new quaternion & omega back in arrays
    q_array(count+1,:) = q_new(:);
    omega_array(count+1,:) = omega_new(:);
    
    omega_rw_array(count+1,:) = omega_rw_new(:);
    
%     q_error_array(count,:) = q_error(:);
    omega_error_array(count,:) = omega_error(:);
    
%     q_d_array(count+1,:) = q_d_new(:);
    
    omega_dot_array(count+1,:) = omega_dot_new(:);
end

figure;
plot(omega_array)
title('Angular Velocity')
legend('Omega_x','Omega_y','Omega_z')
figure;
plot(q_array)
title('Attitude Quaternion')
legend('q_0','q_1','q_2','q_3')
figure;
plot(omega_rw_array)
title('Angular Velocity of RWs')
legend('Omega_x','Omega_y','Omega_z')
% figure;
% plot(q_d_array)
% title('Desired Quaternion')
% legend('q_0','q_1','q_2','q_3')
