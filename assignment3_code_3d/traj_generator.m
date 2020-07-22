function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% nargin
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

persistent waypoints0 traj_time d0 X1 poly 
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    N = size(waypoints, 2)-1; 
    
    
    poly = zeros(7, 8);

    poly(1,:) = ones(1,8);

    x = poly2sym(poly(1,:));

    x_d = diff(x);
    poly(2,2:8) = coeffs(x_d);
    x_dd = diff(x_d);
    poly(3,3:8) = coeffs(x_dd);
    x_ddd = diff(x_dd);
    poly(4,4:8) = coeffs(x_ddd);
    x_dddd = diff(x_ddd);
    poly(5,5:8) = coeffs(x_dddd);
    x_ddddd = diff(x_dddd);
    poly(6,6:8) = coeffs(x_ddddd);
    x_dddddd = diff(x_ddddd);
    poly(7,7:8) = coeffs(x_dddddd); 

    
    zero = diag(poly);
    zero = diag(zero);
    zero = [zero,zeros(7,1)];


    A = zeros(8*N);
    
    B = zeros(8*N, 3);

    

    for i = 1:N

        A((i)*8-7, (1:8)+(i-1)*8) = zero(1,:); 
        B((i)*8-7, :) = waypoints(:,i)';

        A((i)*8-6, (1:8)+(i-1)*8) = poly(1,:); 
        B((i)*8-6, :) = waypoints(:,i+1)';

        if i < N

            A((i)*8-5, (1:16)+(i-1)*8) = [poly(2,:), -zero(2,:)]; 

            A((i)*8-4, (1:16)+(i-1)*8) = [poly(3,:), -zero(3,:)]; 

            A((i)*8-3, (1:16)+(i-1)*8) = [poly(4,:), -zero(4,:)]; 

            A((i)*8-2, (1:16)+(i-1)*8) = [poly(5,:), -zero(5,:)]; 

            A((i)*8-1, (1:16)+(i-1)*8) = [poly(6,:), -zero(6,:)]; 

            A((i)*8, (1:16)+(i-1)*8) = [poly(7,:), -zero(7,:)]; 
        end

    end

    
    A(8*N-5, 1:8) = zero(2,:);
    A(8*N-4, 1:8) = zero(3,:);
    A(8*N-3, 1:8) = zero(4,:);
    A(8*N-2, (1:8)+8*(N-1)) = poly(2,:);
    A(8*N-1, (1:8)+8*(N-1)) = poly(3,:);
    A(8*N  , (1:8)+8*(N-1)) = poly(4,:);


   
for i =1:3
    X1(:,:,i) = reshape(A\B(:,i), 8, N);
end   
    
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
        scale = t/d0(t_index-1);
        xt=[1 scale scale.^2 scale.^3 scale.^4 scale.^5 scale.^6 scale.^7];
        x=xt*X1(:,t_index-1,1);
        y=xt*X1(:,t_index-1,2);
        z=xt*X1(:,t_index-1,3);
        desired_state.pos = [x;
                             y;
                             z];
                         
        vt=[0 1 2*scale 3*scale.^2 4*scale.^3 5*scale.^4 6*scale.^5 7*scale.^6 ];
        vx=vt*X1(:,t_index-1,1);
        vy=vt*X1(:,t_index-1,2);
        vz=vt*X1(:,t_index-1,3);
        desired_state.vel = [vx;
                             vy;
                             vz]/d0(t_index-1);
        at=[0 0 2 3*2*scale 4*3*scale.^2 5*4*scale.^3 6*5*scale.^4 7*6*scale.^5  ]/(d0(t_index-1)).^2;
        ax=at*X1(:,t_index-1,1);
        ay=at*X1(:,t_index-1,2);
        az=at*X1(:,t_index-1,3);
        desired_state.acc = [ax;
                             ay;
                             az];
        
    end

    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

end

