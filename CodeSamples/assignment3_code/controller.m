function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kdy = 10;
Kpy = 0.01;

Kpz = 100.0;
Kdz = 25;

Kpr = 5000.0;
Kdr = 100;

%y_error = des_state.pos(1) - state.pos(1);
%ydot_error = des_state.vel(1) - state.vel(1);
z_error = des_state.pos(3) - state.pos(3);
zdot_error = des_state.vel(3) - state.vel(3);

%roll_angle = -(des_state.acc(1) + (Kdy*ydot_error) + (Kpy*y_error))/params.gravity;

%roll_error = roll_angle - state.rot;
%rolldot_error = -state.omega;
%rollddot = 0;

F = params.mass*(params.gravity + des_state.acc(3) + (Kdz*zdot_error) + (Kpz*z_error));
%u2 = params.Ixx*(rollddot + (Kdr*rolldot_error) + (Kpr*roll_error));
% Thrust
%F = 0;

% Moment
M = zeros(3,1);

% =================== Your code ends here ===================

end
