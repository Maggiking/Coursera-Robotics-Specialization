function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;


% FILL IN YOUR CODE HERE
Kp = 100;
Kv = 15;
e_z = s_des(1)-s(1);
e_v = s_des(2)-s(2);
u = params.mass*(Kp*e_z+Kv*e_v+params.gravity);

end

