function trajectory_state = trajectory_planner(question, waypoints, max_iter, waypoint_times, time_step)

% Input parameters
% 
%   question: Which question we are on in the assignment
%
%   waypoints: Series of points in [x; y; z; yaw] format
%
%   max_iter: Number of time steps
%
%   waypoint_times: Time we should be at each waypoint
%
%   time_step: Length of one time_step
%
% Output parameters
%
%   trajectory_sate: [15 x max_iter] output trajectory as a matrix of states:
%   [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];
%
%************  TRAJECTORY PLANNER ************************

% Write code here
trajectory_state = zeros(15, max_iter);
% height of 15 for: [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

if question == 2 || question == 41 || question == 42 || question == 43 || question == 44 || question == 45 || question == 52 || question == 53
    % Sample code for hover trajectory
    current_waypoint_number = 1;
    for iter = 1:max_iter
        if (current_waypoint_number < length(waypoint_times))
            if((iter * time_step) > waypoint_times(current_waypoint_number + 1))
                current_waypoint_number = current_waypoint_number + 1;
            end
        end
            
        trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
        trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
    end
elseif question == 3
    current_waypoint_number = 1;
    for iter = 1:max_iter
        
        if (current_waypoint_number < length(waypoint_times))
            if((iter * time_step) > waypoint_times(current_waypoint_number + 1))
                current_waypoint_number = current_waypoint_number + 1;
            end
        end
           
        trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
        trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
        
        % Velocity Ramp for z
        a = 0.025;
        if current_waypoint_number < length(waypoint_times) / 4
            trajectory_state(6,iter) = a * iter * time_step;
        elseif current_waypoint_number < 3 * length(waypoint_times) / 4
            trajectory_state(6,iter) = a * (10 - iter * time_step);
        else
            trajectory_state(6,iter) = a * (-20  + iter * time_step);
        end
        
    end
elseif question == 58 || question == 59
    current_waypoint_number = 1;
    for iter = 1:max_iter
        if (current_waypoint_number < length(waypoint_times))
            if((iter * time_step) > waypoint_times(current_waypoint_number + 1))
                current_waypoint_number = current_waypoint_number + 1;
            end
        end
            
        trajectory_state(1:3,iter) = waypoints(1:3,current_waypoint_number);
        trajectory_state(4:6,iter) = zeros(3, 1);   % Zero velocity
        trajectory_state(9,iter) = waypoints(4,current_waypoint_number);
    end
end

end
