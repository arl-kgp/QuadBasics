function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

Kdy = 10;
Kpy = 0.01;

Kpz = 100.0;
Kdz = 25;

Kpr = 5000.0;
Kdr = 100;

y_error = des_state.pos(1) - state.pos(1);
ydot_error = des_state.vel(1) - state.vel(1);
z_error = des_state.pos(2) - state.pos(2);
zdot_error = des_state.vel(2) - state.vel(2);

roll_angle = -(des_state.acc(1) + (Kdy*ydot_error) + (Kpy*y_error))/params.gravity;

roll_error = roll_angle - state.rot;
rolldot_error = -state.omega;
rollddot = 0;

u1 = params.mass*(params.gravity + des_state.acc(2) + (Kdz*zdot_error) + (Kpz*z_error));
u2 = params.Ixx*(rollddot + (Kdr*rolldot_error) + (Kpr*roll_error));
%u2 = 0;
% FILL IN YOUR CODE HERE

end

