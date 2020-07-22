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
psi_des=des_state.yaw;
rdes=des_state.yawdot;

xd=des_state.vel(1);
yd=des_state.vel(2);

xp=des_state.pos(1);
yp=des_state.pos(2);

p= state.omega(1) ;
 q= state.omega(2) ;
 r= state.omega(3) ;
 kpx=2000*0.01;
 kdx=8000*0.3;
  kpy=2000*0.01;
 kdy=8000*0.3;
kp=20*0.01;
kd=2000*0.3;
kpp=50*0.01;
kdp=650*0.3;
kpt=50*0.01;
kdt=650*0.3;
kps=2*0.01;
kds=500*0.3;
%   Using these current and desired states, you have to compute the desired
%   controls


n=des_state.acc/norm(des_state.acc);
t=des_state.pos/norm(des_state.pos);
b=cross(t,n);
if (all(des_state.pos-state.pos)>0)
    ep=(des_state.pos-state.pos);
    %ep=((des_state.pos-state.pos)'*n)*n+((des_state.pos-state.pos)'*b)*b;
else
    ep=((des_state.pos-state.pos)'*n)*n+((des_state.pos-state.pos)'*b)*b;
    %ep=(des_state.pos-state.pos);
end
phi=state.rot(1);
theta=state.rot(2);
psi=state.rot(3);
pdes=0;
qdes=0;
m=params.mass;
g=params.gravity;
Z_ddot=des_state.acc(3);
Y_ddot=des_state.acc(2);
X_ddot=des_state.acc(1);
F=m*(Z_ddot+kd*(des_state.vel(3)-state.vel(3))+kp*(ep(3)))+m*g;
% =================== Your code goes here ===================
x_dd=X_ddot+kdx*(xd-state.vel(1))+kpx*(ep(1));
y_dd=Y_ddot+kdy*(yd-state.vel(2))+kpy*(ep(2));
% Thrust

phi_des=(x_dd*sin(psi)-y_dd*cos(psi));
theta_des=(x_dd*cos(psi)+y_dd*sin(psi));

% Moment
M = zeros(3,1);
M=[kpp*(phi_des-phi)+kdp*(pdes-p);kpt*(theta_des-theta)+kdt*(qdes-q);kps*(psi_des-psi)+kds*(rdes-r)];
% =================== Your code ends here ===================

end
