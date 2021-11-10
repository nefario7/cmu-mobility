function [] = state_machine(question)

% Operations for the quadcopter
% 1. Idle 
% 2. Take-off
% 3. Hover
% 4. Tracking
% 5. Land

% To run state machine for different questions:
% Question 4 - state_machine(4)
% Question 5 (No heading) - state_machine(50)
% Question 5 (With heading) - state_machine(51)

if question == 4
    disp("Running State Machine")
    operations = [42, 43, 44, 43, 45];
    operation_times = [-1, 5, -1, 5, -1];
elseif question == 50
    operations = [52, 53, 58];
    operation_times = [-1, 5, -1];    
elseif question == 51
    operations = [52, 53, 59];
    operation_times = [-1, 5, -1];  
else
    disp("Enter a valid question number (4 or 5)!")
    return;
end

total_time = 0;
last_state = zeros(16,1);
actual = ones(15, 1);
desired = ones(15, 1);

for i = 1:length(operations)
    disp(" > Running " + operations(i))
    
    [actual_state_matrix, actual_desired_state_matrix, final_state, time] = state_machine_main(operations(i), last_state, operation_times(i));

    total_time = total_time + time;
    last_state = final_state;
    actual = [actual(:, 1:end - 1) actual_state_matrix(:, 2:end - 1)];
    desired = [desired(:, 1:end - 1) actual_desired_state_matrix(:, 2:end - 1)];
end

time_step = 0.005;
total_time_vec = 0:time_step:total_time - (length(operations) * time_step * 2);

% end
% disp(size(actual))
% disp(size(desired))
% disp(size(total_time_vec))

plot_quadrotor_errors(actual, desired, total_time_vec)
disp("Done!")
end