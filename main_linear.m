clc; clear; close all;

% Title: Safe Output-Feedback Control via SDRE and Barrier States
% Authors: Tochukwu Ogri and Trivikram Satharasi
% Emails: tochukwu.ogri@ufl.edu, t.satharasi@ufl.edu
% Date: 2025-09-12
%
% Description:
% Simulates a nonlinear system with Barrier and Robustified SDRE controller.
% Saves results in data/<systemType>/<case_name>/ folder.
% Supports multiple cases: BarrierOnly, BarrierAndRobust.

%% Linear Simulation
systemType = "linear";
root_folder = fullfile("data", systemType);

%% Simulation parameters

% Initial states
x0 = [3;3];
x0_hat = [3.25;3.25];

% Obstacle parameters
obs_center = [1.4; 1.8];
obs_radius = 0.5;
fun_A = @(x) [1, -5; 0, -1]; % linear SDC matrix
fun_G =  @(x) [0;1]; % Linear

% Devansh RCBF parameters
b_minus (1,1) = 1;
b_plus (1,1) = 1;

%% Run all controller cases
% No BaS
nb = SDREBasedControl(x0, x0_hat, fun_G, fun_A, systemType);
nb.run();

% With BaS and robust Bas
wb = SDREBasedWithBarrier(x0, x0_hat, obs_center, obs_radius, fun_G, fun_A, systemType);
wb.run();

% Devansh Controller for comparison
db = DevanshControl(x0, x0_hat, obs_center, obs_radius, fun_G, fun_A, b_minus, b_plus, systemType);
db.run();

% Case configurations
cases = {
    struct("name", "NoBarrier", "label", "No Barrier", ...
           "color", [0 0.4470 0.7410]) % blue
    struct("name", "BarrierOnly", "label", "Barrier Only", ...
           "color", [0.1330 0.5450 0.1330]) % forest green
    struct("name", "BarrierAndRobust", "label", "Barrier + Robust", ...
           "color", [0.9290 0.6940 0.1250]) % golden orange
    struct("name", "DevanshRCBF", "label", "Devansh RCBF", ...
           "color", [0.4940 0.1840 0.5560]) % purple
};

%% Create figure for system trajectories while avoiding obstacle
figure;
hold on;
title("State Trajectories vs Estimates (x_1 vs x_2)");
xlabel("x_1"); ylabel("x_2");
axis equal; grid on;

% Draw obstacle
theta = linspace(0, 2*pi, 200);
x_obs = obs_center(1) + obs_radius * cos(theta);
y_obs = obs_center(2) + obs_radius * sin(theta);
fill(x_obs, y_obs, 'r', 'FaceAlpha', 0.2, ...
     'EdgeColor', 'r', 'LineWidth', 2, ...
     'DisplayName', 'Obstacle');

% Loop through all cases and plot both x and x_hat
for i = 1:numel(cases)
    case_name = cases{i}.name;
    label = cases{i}.label;
    color = cases{i}.color;

    % Load actual and estimated state data
    x_path = fullfile(root_folder, case_name, "x_data.dat");
    x_hat_path = fullfile(root_folder, case_name, "x_hat_data.dat");

    if isfile(x_path) && isfile(x_hat_path)
        x = load(x_path);
        x_hat = load(x_hat_path);

        % Plot actual state
        plot(x(:,1), x(:,2), '-', ...
            'Color', color, ...
            'LineWidth', 1.8, ...
            'DisplayName', sprintf('%s - True', label));

        % Plot estimated state
        plot(x_hat(:,1), x_hat(:,2), '--', ...
            'Color', color * 0.7 + 0.3, ... % lighten estimate line
            'LineWidth', 1.2, ...
            'DisplayName', sprintf('%s - Estimate', label));
    else
        warning("Data files not found for case: %s", case_name);
    end
end

legend('Location', 'bestoutside');

%% Create figure for state estimation errors
figure;
hold on; grid on;
title("State Estimation Error");
xlabel("Time [s]");
ylabel("Estimation Error");

% Loop through all cases and plot error
for i = 1:numel(cases)
    case_name = cases{i}.name;
    label = cases{i}.label;
    color = cases{i}.color;

    % Load error data
    e_path = fullfile(root_folder, case_name, "e_data.dat");
    t_path = fullfile(root_folder, case_name, "t_data.dat");
    
    if isfile(e_path)
        e = load(e_path);  % columns correspond to state errors
        t = load(t_path); % timestep 

        % Plot each state error
        for j = 1:size(e,2)
            plot(t, e(:,j), '-', ...
                'Color', color * (1 - 0.3*(j-1)), ...
                'LineWidth', 1.5, ...
                'DisplayName', sprintf('%s - e_{x%d}', label, j));
        end
    else
        warning("Error data file not found for case: %s", case_name);
    end
end

legend('Location','bestoutside');