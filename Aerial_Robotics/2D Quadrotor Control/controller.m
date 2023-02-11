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

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE

kvz = 50;
kvy = 50;
kvp = 50;
kpz = 200;
kpp = 200;
kpy = 200;

e_z = des_state.pos(2)-state.pos(2);
e_dz = des_state.vel(2)-state.vel(2);

e_y = des_state.pos(1)-state.pos(1);
e_dy = des_state.vel(1)-state.vel(1);
e_ddy = des_state.acc(1)+params.gravity*state.rot;
e_dddy= params.gravity*state.omega;

phi = -1/params.gravity*(des_state.acc(1) + kvy*e_dy + kpy*e_y);
dphi = -1/params.gravity*(kvy*e_ddy + kpy*e_y);
ddphi= -1/params.gravity*(kvy*e_dddy + kpy*e_ddy);
e_p  = phi - state.rot;
e_dp = dphi - state.omega;

u1 = params.mass * (params.gravity + des_state.acc(2) + kvz*e_dz + kpz*e_z);
u2 = params.Ixx * (ddphi + kvp*e_dp + kpp*e_p);

end

