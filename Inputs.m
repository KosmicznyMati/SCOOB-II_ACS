addpath 'C:\Users\Mat\Desktop\FYP\matlab & simulink\Functions'

% INPUTS

% Constants

% Spacecraft
J = [0.0298629 0.0001670 0.000035;
     0.0001670 0.0063248 0.0000685;
     0.000035  0.0000685 0.0306925];                % Moment of inertia of spacecraft kg-m^2
% J = eye(3);
J_inv = inv(J);

% Reaction Wheels
J_RW = [0.00000820  0   0;
        0   0.00000820  0;
        0   0   0.00001281];                        % Moment of inertia of reaction wheel kg-m^2
    
rw_align = eye(3);                                  % Reaction wheels alignment matrix
k_n = 543;                                          % BLDC speed constant in min^-1/V
rw_conversion = k_n*3.141592/30;                   % rad/s/V conversion factor

m = [0.2 0.2 0.2]';
B = [1 0 0]';
desaturation_mode = 0;
omega_rw_saturated = 10000;
omega_rw_desaturated = 20;
saturation_dir = [0; 0; 0];

% Controller
K_p = 0.1;                                          % Proportional gain; 0.2 for quaternion control;
K_d = 0.1;                                          % Differential gain; 0.1 for quaternion control; (-) depending on error calc.
K_p_w = 0.7;                                        %  0.7 for omega control 
K_d_w = -0.005;                                     % -0.005 for omega control

% Initial Conditions
omega_rw_initial = [0 0 0]';                        % Initial angular velocity of RWs

omega_initial   = [0.1 0.1 0.1]';%.1 0.01 0.01]';                % Initial angular velocity of system
Direction_vector_q = [0 1 0]';                      % Initial direction vector
alpha = 30;                                          % Initial angle in degrees

unit_u_q = Unit(Direction_vector_q);                % Unitising the direction vector
q_initial = [cosd(alpha); unit_u_q*sind(alpha)];    % Initial quaternion

omega_dot_initial = [0 0 0]';                       % Initial angular acceleration

% Desired Conditions
omega_d         = [0 0 0]';                         % Desired omega; if non-zero, timestep must be changed to 0.0001s
Direction_vector_p = [1 0 0]';                      % Initial direction vector
beta  = 0;                                         % Desired angle in degrees

unit_u_p = Unit(Direction_vector_p);                % Unitising the direction vector
q_d = [cosd(beta); unit_u_p*sind(beta)];            % Desired quaternion

% q_dot_d = [0.1 0.1 0 0]';                         % Rate of change of desired quaternion
omega_dot_d = [0 0 0]';                             % Desired angular acceleration