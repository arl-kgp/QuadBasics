function [ desired_state ] = traj_generator(t, ~, waypoints)
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
% if nargin > 2   %Initialisation
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);   %distance per segment
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else        %Simulation time
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);   %find the index of the closest tabulated trajectory time corresponding to the current time
%     if(t_index > 1) %Exception for the first time point
%         t = t - traj_time(t_index-1);
%     end         
%     if(t == 0)  %Exception for time=0
%         desired_state.pos = waypoints0(:,1);
%     else    %For any other time, make a scalar progression between points
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

persistent waypoints0 traj_time d0 Cx Cy Cz
if nargin > 2   %Initialisation
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);   %distance per segment
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    
    Cx=computeCoefs(waypoints0(1,:));
    Cy=computeCoefs(waypoints0(2,:));
    Cz=computeCoefs(waypoints0(3,:));
    
    for i=1:size(waypoints0,2)
        plot3(waypoints0(1,i),waypoints0(2,i),waypoints0(3,i),'g*')
        hold on
    end
    
else        %Simulation time
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1)-1;   %find the index of the closest tabulated trajectory time corresponding to the current time
    
    if (t_index == 0)
        t_index = 1;
    end
    
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = 0*waypoints0(:,1);
        desired_state.acc = 0*waypoints0(:,1);
    else
        %create scaled time, value between 0 to 1
        scale = (t-traj_time(t_index))/d0(t_index);
        
        index = (t_index-1)*8+1:t_index*8;
        
        %calculate position:
        t0 = Polynom(8,0,scale);
        desired_state.pos = [Cx(index)'*t0; Cy(index)'*t0; Cz(index)'*t0];
        
        %calculate velocity:
        t1 = Polynom(8,1,scale)';
        desired_state.vel = [Cx(index)'*t1; Cy(index)'*t1; Cz(index)'*t1].*(1/d0(t_index));
        
        %calculate acceleration:
        t2 = Polynom(8,2,scale)';
        desired_state.acc = [Cx(index)'*t2; Cy(index)'*t2; Cz(index)'*t2].*(1/d0(t_index)^2);
    end
    
    % leave desired yaw and yawdot at zero
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    
end
end

%%Functions
function [C]=computeCoefs(w)
%computes the coefficients for the polynomials that defines the trajectory.
%The problem to solve is A*Coeffs=b, where A contains the constraints, b
%contains the waypoints and Coeffs is the matrix of coefficients. It must
%be taken into account that it has to be solved for x, y and z.

    n=size(w,2)-1; %Number of polynomials -1

    %initialise matrixes:
    A=zeros(8*n,8*n);

    b = zeros(1,8*n);
    for i=1:n
        b(1,i) = w(i);
        b(1,i+n) = w(i+1);
    end
    %Construction of A

    %Derivative constraint at node 1 P1_dot(t=0)=0, P1_dotdot(t=0)=0...
    %& Derivative constraint at node n Pn_dot(t=0)=0, Pn_dotdot(t=0)=0...
    for k=1:3   %loop through the three derivatives
        A(2*n+k,1:8) = Polynom(8,k,0); %Constraint at node 1
        A(2*n+3+k,(end-7):end) = Polynom(8,k,1); %Constraint at node n
    end

    for j=1:n  %loop through all waypoints

        %position constraint: P(t=0)=wi P(t=1)=wi+1
        A(j,((j-1)*8)+1:j*8) = Polynom(8,0,0);
        A(j+n,((j-1)*8)+1:j*8) = Polynom(8,0,1);



        %Derivative constraints at internal waypoints
        %Pi_dot(t=1)=Pi+1_dot(t=0) ...
        if j~=1
            for k=1:6 %loop through all derivatives
                A(2*n+6+(j-2)*6+k, (j-2)*8+1:((j-2)*8+n*n)) = [Polynom(8,k,1) -Polynom(8,k,0)];
            end
        end
    end
    

    %Compute the coefficients
    C=A\b';
end




