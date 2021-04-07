addpath 'C:\Users\Mat\Desktop\FYP\matlab & simulink\Functions'

% INPUTS

% Constants 

% Spacecraft
J = [0.0298629 0.0001670 0.000035;
     0.0001670 0.0063248 0.0000685;
     0.000035  0.0000685 0.0306925];     % Moment of inertia of spacecraft kg-m^2
% J = [10 0 0; 0 10 0; 0 0 10];
J_inv = inv(J);

% Reaction Wheels
J_RW = [0.00000820  0   0;
        0   0.00000820  0;
        0   0   0.00001281];           % Moment of inertia of reaction wheel kg-m^2
    
rw_align = eye(3);                  % Reaction wheels alignment matrix

% Controller
K_p = 0.002;                          % Proportional gain; 0.2 for quaternion control; 0.002 for omega control 
K_d = -0.001;                         % Differential gain; -0.1 for quaternion control; -0.001 for omega control 

% Initial Conditions
omega_rw_initial = [0 0 0]';        % Initial angular velocity of RWs

omega_initial   = [0 0 0]';         % Initial angular velocity of system
Direction_vector_q = [1 0 0]';      % Initial direction vector
alpha = 0;                         % Initial angle in degrees

unit_u_q = Unit(Direction_vector_q);% Unitising the direction vector
q_initial = [cosd(alpha); unit_u_q*sind(alpha)];    % Initial quaternion

omega_dot_initial = [0 0 0]';       % Initial angular acceleration

% Desired Conditions
omega_d         = [0.01 0.01 0.01]';                         % Desired omega
Direction_vector_p = [1 0 0]';                          % Initial direction vector
beta  = 0;                                             % Desired angle in degrees

unit_u_p = Unit(Direction_vector_p);                    % Unitising the direction vector
q_d = [cosd(beta); unit_u_p*sind(beta)];              % Desired quaternion

% q_dot_d = [5 5 5 5]';                 % Rate of change of desired quaternion

omega_dot_d = [0 0 0]';             % Desired angular acceleration