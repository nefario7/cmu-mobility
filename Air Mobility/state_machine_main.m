function [actual_state_matrix, actual_desired_state_matrix, final_state, time_final] = state_machine_main(stage, initial_state, stage_time)
%% Set up quadrotor physical parameters
params = struct(...
    'mass',                   0.770, ...
    'gravity',                9.80665, ...
    'arm_length',           0.1103, ...
    'motor_spread_angle',     0.925, ...
    'thrust_coefficient',     8.07e-9, ...
    'moment_scale',           1.3719e-10, ...
    'motor_constant',        36.5, ...
    'rpm_min',             3000, ...
    'rpm_max',            20000, ...
    'inertia',            diag([0.0033 0.0033 0.005]),...
    'COM_vertical_offset',                0.05);

time_initial = 0; 
time_step = 0.005;
time_final = 10;
if stage_time > 0
    time_final = stage_time;
end
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);
max_error_threshold = 10e-4;

%% Get the waypoints for this specific stage
[waypoints, waypoint_times] = lookup_waypoints(stage);

%% Create the state vector
state = initial_state;

%% Create a trajectory consisting of desired state at each time step
trajectory_matrix = trajectory_planner(stage, waypoints, max_iter, waypoint_times, time_step);
% [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

%% Create a matrix to hold the actual state at each time step
actual_state_matrix = zeros(15, max_iter);
actual_state_matrix(1:12, 1) = state(1:12, 1);
% [x; y; z; xdot; ydot; zdot; phi; theta; psi; phidot; thetadot; psidot; xacc; yacc; zacc];

%% Create a matrix to hold the actual desired state at each time step
actual_desired_state_matrix = zeros(15, max_iter);  

%% Loop through the timesteps and update quadrotor
done = 0;
count = 0;
start = 1;
while ~done
%     disp("Start " + start + " - " + "Max_iter " + (max_iter - 1) * time_step)
    for iter = start:max_iter - 1
        % convert current state to stuct for control functions
        current_state.pos = state(1:3);
        current_state.vel = state(4:6);
        current_state.rot = state(7:9);
        current_state.omega = state(10:12);
        current_state.rpm = state(13:16);
            
        % Get desired state from matrix, put into struct for control functions
        desired_state.pos = trajectory_matrix(1:3,iter);
        desired_state.vel = trajectory_matrix(4:6,iter);
        desired_state.rot = trajectory_matrix(7:9,iter);
        desired_state.omega = trajectory_matrix(10:12,iter);
        desired_state.acc = trajectory_matrix(13:15,iter);
        
        % Get desired acceleration from position controller
        [F, desired_state.acc] = position_controller(current_state, desired_state, params, stage);
    
        % Computes desired pitch and roll angles
        [desired_state.rot, desired_state.omega] = attitude_planner(desired_state, params);
    
        % Get torques from attitude controller
        M = attitude_controller(current_state, desired_state, params, stage);
    
        % Motor model
        [F_actual, M_actual, rpm_motor_dot] = motor_model(F, M, current_state.rpm, params);
        
        % Get the change in state from the quadrotor dynamics
        timeint = time_vec(iter:iter+1);
        [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F_actual, M_actual, rpm_motor_dot), timeint, state);
        state    = xsave(end, :)';
        acc  = (xsave(end,4:6)' - xsave(end-1,4:6)')/(tsave(end) - tsave(end-1));
    
        % Update desired state matrix
        actual_desired_state_matrix(1:3, iter+1) =  desired_state.pos;
        actual_desired_state_matrix(4:6, iter+1) = desired_state.vel;
        actual_desired_state_matrix(7:9, iter+1) = desired_state.rot;
        actual_desired_state_matrix(10:12, iter+1) = desired_state.omega;
        actual_desired_state_matrix(13:15, iter+1) = desired_state.acc;
    
        % Update actual state matrix
        actual_state_matrix(1:12, iter+1) = state(1:12);
        actual_state_matrix(13:15, iter+1) = acc;  
    
    end

    if stage ~= 43      % Running error check for operations other than hovering, as hovering has fixed time
        errors = actual_state_matrix(1:3,end) - actual_desired_state_matrix(1:3, end);
        if abs(errors(1, 1)) < max_error_threshold && abs(errors(2,1)) < max_error_threshold && abs(errors(3, 1)) < max_error_threshold
            disp("System stable, moving to next operation!")
            done = 1;
        else
            if count == 5       % If system not stable even after 5 runs, stop
                done = 1;
            else
                disp("System not stable yet, running operation again.")
                % Extra time for stabilization
                extra = 2;
                
                % Continue from the last point
                start = max_iter;
                max_iter = max_iter + extra / time_step;

                % Increase the total time and time vector
                time_final = time_final + extra;
                time_vec = time_initial:time_step:time_final;

                actual_state_matrix(1, max_iter) = 0;
                actual_desired_state_matrix(1, max_iter) = 0; 
                trajectory_matrix(1, max_iter) = 0;
                for i = iter+1:max_iter
                    trajectory_matrix(:, i) = trajectory_matrix(:, iter);
                end

                % Counter to stop after 5 stabilization attempts
                count = count + 1;
            end
        end
    else
        done = 1;
    end
    
    final_state = state;
end

