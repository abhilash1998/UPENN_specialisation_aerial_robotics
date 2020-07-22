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
kpy=3000*.01;
kdy=1000*.3;
kpz=3000*.01;
kdz=1000*.3;
kpp=.01*.01;
kdp=17270*3;


m=params.mass;
e=des_state.pos-state.pos;
ev=des_state.vel-state.vel;
g=params.gravity;
Ixx=params.Ixx;
phi=state.rot;

phic=(-1*(des_state.acc(1)+kdy*ev(1)+kpy*e(1)))/g;

persistent pephi;
if isempty(pephi)
        pephi = 0;
end
ephi=phic-phi;
ephidot=ephi-pephi;

u1=m*(g+des_state.acc(2)+kdz*ev(2)+kpz*e(2));

u2=Ixx*(kdp*ephidot+kpp*ephi);

pephi=ephi;





% FILL IN YOUR CODE HERE

end

