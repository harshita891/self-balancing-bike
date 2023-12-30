function mainq()
    % Define simulation parameters
    dt = 0.01;
    simulation_time = 100;
    track_width = 10; % Set the track width
    track_length = 20; % Set the track length

    % Run the simulation
    [trajectory_x, trajectory_y, gyroscope_data, GPS_data] = simulateBike(dt, simulation_time, track_width, track_length);

    % Plot the bike's trajectory
    plotTrajectory(trajectory_x, trajectory_y);

    % Display gyroscope and GPS data
    disp('Gyroscope Data:');
    disp(gyroscope_data);
    disp('GPS Data:');
    disp(GPS_data);
end

function [trajectory_x, trajectory_y, gyroscope_data, GPS_data] = simulateBike(dt, simulation_time, track_width, track_length)
    % Define the bike's parameters
    m = 20; % mass of the bike (kg)
    g = 9.8; % acceleration due to gravity (m/s^2)
    wheel_radius = 0.3; % radius of the wheels (m)
    motor_torque = 5; % maximum torque of the motor (Nm)
    chain_ratio = 2;

    % Define the control gains
    Kp = 1;
    Ki = 0.5;
    Kd = 0.1;

    % Initialize arrays to store trajectory data
    trajectory_x = zeros(1, round(simulation_time / dt));
    trajectory_y = zeros(1, round(simulation_time / dt));

    % Initialize gyroscope and GPS data arrays
    gyroscope_data = zeros(3, round(simulation_time / dt) + 1); % Add +1 here
    GPS_data = zeros(3, round(simulation_time / dt));

    % Initialize other variables as needed

    % Initialize the bike's initial position, orientation, and velocity
    x = 0;
    y = 0;
    theta = 0;
    v = 0;

    % Initialize control algorithm variables
    integral = 0;

    % Initialize motor_speed
    motor_speed = 10;

    % Simulate the bike's dynamics
    for t = 2:round(simulation_time / dt) + 1 % Start from 2 to avoid t-1 in the first iteration
        % Calculate the error
        error = 0; % Replace with actual error calculation if needed

        % Update the integral term
        integral = integral + error * dt;

        % Calculate the control input
        u = Kp * error + Ki * integral - Kd * v;

        % Calculate the acceleration due to the wind resistance
        a_wind = 0; % Replace with actual wind resistance calculation

        % Calculate the acceleration due to the friction
        a_friction = 0; % Replace with actual friction calculation

        % Calculate the total acceleration
        a_total = (u / chain_ratio - a_wind - a_friction) / (wheel_radius * 2);

        % Update the velocity
        v = v + a_total * dt;

        % Update the position
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;

        % Update the orientation
        theta = theta + (v / wheel_radius) * dt;

        % Update gyroscope data
        motor_speed = 0; % Replace with actual calculation for motor_speed
        gyroscope_data(:, t) = [gyroscope_data(1, t-1) + (motor_torque * dt) / (m * g);
                                gyroscope_data(2, t-1) + (motor_speed * dt) / (m * g);
                                gyroscope_data(3, t-1) + (motor_torque * dt) / (m * g)];

        % Update GPS data
        GPS_data(:, t-1) = [x; y; theta]; % Update at t-1 to match indexing

        % Update the position in trajectory arrays
        trajectory_x(t-1) = x;
        trajectory_y(t-1) = y;

        % Check for collisions with obstacles
        if (x < 0 || x > track_width || y < 0 || y > track_length)
            disp('Collision!');
            break;
        end
    end
end

function plotTrajectory(trajectory_x, trajectory_y)
    % Plot the bike's trajectory
    figure;
    plot(trajectory_x, trajectory_y, 'ob', 'Linewidth', 2);
    xlabel('X');
    ylabel('Y');
    title('Bike''s Trajectory');
end
