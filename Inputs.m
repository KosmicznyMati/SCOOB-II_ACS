addpath 'C:\Users\Mat\Desktop\FYP\matlab & simulink\Functions'

% INPUTS

% Constants 
% J = [0.0298629 0.0001670 0.000035; 0.0001670 0.0063248 0.0000685;
%  0.000035 0.0000685 0.0306925];     % Moment of inertia of spacecraft kg-m^2
J = [10 0 0; 0 10 0; 0 0 10];
J_inv = inv(J);
q_identity = [1 0 0 0]';            % Identity quaternion

K_p = 0;                          % Proportional gain 0.2
K_d = 0;                         % Differential gain -0.1


% Initial Conditions
omega_initial   = [0 0.01 0.01]';   % Initial angular velocity of system
Direction_vector_q = [1 0 0]';      % Initial direction vector
alpha = 90;                         % Initial angle in degrees

unit_u_q = Unit(Direction_vector_q);% Unitising the direction vector
q_initial = [cosd(alpha/2); unit_u_q*sind(alpha/2)];    % Initial quaternion

% Desired Conditions
omega_d         = [0 0 0]';                             % Desired omega
Direction_vector_p = [1 0 0]';                          % Initial direction vector
beta  = 0;                                              % Desired angle in degrees

unit_u_p = Unit(Direction_vector_p);                    % Unitising the direction vector
p = [cosd(beta/2); unit_u_p*sind(beta/2)];              % Desired quaternion

