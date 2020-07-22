function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
persistent prev_e;
if isempty(prev_e)
        prev_e = 0;
    end
kp=200*200;
kd=200*1000;
e=s_des-s;
ev=e(1)-prev_e;
g=params.gravity;
m=params.mass;
u = m*(kp*e(1)*0.01+kd*ev*.3+g);
prev_e=e(1);

% FILL IN YOUR CODE HERE


end

