function [waypoints, waypoint_times] = lookup_waypoints(question)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoints, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Defined Waypoints
% 2 - Waypoints for question 2
% 3 - Waypoints for question 3
% 41, 42, 43, 44, 45 - Waypoints for idle, take-off, hover, tracking and landing (respectively)
% 52, 53, 58, 59 - Waypoitns for take-off, hover, tracking (no heading) and tracking (with heading) (respectively)

% Waypoints
if question == 2
    % Waypoints for hover at z = 0.5m and x-direction increments of 0.1m
    waypoints = [
        0 0 0.1 0.2 0.3; 
        0 0 0 0 0; 
        0 0.5 0.5 0.5 0.5; 
        0 0 0 0 0
        ];
    % Waypoint times
    waypoint_times = [0 2 4 6 8];

elseif question == 3 
    % Take-off from a starting location
    % Go to a fixed height of 1m
    % Return to the ground
    waypoints = [
        zeros(1, 200);  
        zeros(1, 200);
        0.01:0.01:1 0.99:-0.01:0;
        zeros(1, 200)
        ]; 
    waypoint_times = [linspace(0, 20, 200)];    %Time steps spread over a period of 20 seconds

elseif question == 41
    % Waypoints for idle state
    waypoints = [
        0; 
        0; 
        0; 
        0 ];
    % Waypoint times
    waypoint_times = [2];

elseif question == 42
    % Waypoints for take-off to z = 1m
    waypoints = [
        0; 
        0; 
        1; 
        0 ];
    % Waypoint times
    waypoint_times = [2];

elseif question == 43
    % Waypoints for hover at z = 1m for 5 sec
    waypoints = [
        0; 
        0; 
        1; 
        0
        ];
    % Waypoint times
    waypoint_times = [5];

elseif question == 44 
    % Go to a fixed height of 1m in x and y-axis and 2m in z-axis
    waypoints = [
        0.02:0.02:1 0.98:-0.02:0;  
        0.02:0.02:1 0.98:-0.02:0;
        1.02:0.02:2 1.98:-0.02:1;
        zeros(1, 100)
        ]; 
    waypoint_times = [linspace(0, 10, 100)];    %Time steps spread over a period of 20 seconds

elseif question == 45
    % Waypoints for landing to z = 0m
    waypoints = [
        0;  
        0;
        0;
        0
        ]; 
    waypoint_times = [2]; 

elseif question == 52
    % Waypoints for take-off to z = 0.1m
    waypoints = [
        0; 
        0; 
        1; 
        0 ];
    % Waypoint times
    waypoint_times = [2];
elseif question == 53
    % Waypoints for hover at z = 1m with no velocity
    waypoints = [
        0 0; 
        0 0; 
        1 1; 
        0 0
        ];
    % Waypoint times
    waypoint_times = [5 5];
elseif question == 58
    % Waypoints for tracking z = 1.1m
    waypoints = [
        0 0; 
        0 0; 
        1.1 1.1; 
        0 0
        ];
    % Waypoint times
    waypoint_times = [5 5];
elseif question == 59
    % Waypoints for tracking z = 1.1m and heading 15 degrees
    waypoints = [
        0 0; 
        0 0; 
        1.1 1.1; 
        deg2rad(15) deg2rad(15)
        ];
    % Waypoint times
    waypoint_times = [5 5];
end

end
