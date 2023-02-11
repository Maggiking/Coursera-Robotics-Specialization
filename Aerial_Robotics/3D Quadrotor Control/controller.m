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

% Thurst
kd = [100;100;100];
kp = [100;100;100];
e_p = (des_state.pos - state.pos);
e_d = (des_state.vel - state.vel);
r_dd = des_state.acc + diag(kd*e_d' + kp*e_p');
F = params.mass * (params.gravity + r_dd(3));

% Moment
kp_rot   = [200;200;200]/50;
kd_rot   = [200;200;200]/50;

phi_des = 1/params.gravity * (r_dd(1)*sin(des_state.yaw) - r_dd(2)*cos(des_state.yaw));
th_des  = 1/params.gravity * (r_dd(1)*cos(des_state.yaw) + r_dd(2)*sin(des_state.yaw));

rot_des = [phi_des;th_des;des_state.yaw;];
om_des  = [0;0;des_state.yawdot];
e_r    = rot_des - state.rot;
e_o    = om_des - state.omega;
M = diag(kd_rot*e_o' + kp_rot*e_r');

% =================== Your code ends here ===================

end