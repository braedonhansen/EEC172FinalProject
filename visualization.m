% Reads a 7-column CSV of [Distance_in, Gyro_X_dps, Gyro_Y_dps, Gyro_Z_dps, Accel_X_g, Accel_Y_g, Accel_Z_g]
% and produces a set of 2D (x,y) points by:
% Integrating Gyro_Z (dps) to get yaw over time,
% Converting each Distance reading into (x,y) in the horizontal plane,
% Integrating Accel_X/Accel_Y to get the sensor’s planar displacement (after converting from g to m/s^2),
% Offsetting each scan point by that displacement.

csvFile = 'C:\Users\Braedon\Downloads\PLEASE.csv'; % change for your .csv

movement = 1;

% Sampling interval (seconds) between successive points (e.g. 0.05 for 20 Hz)
dt = 0.05;

% Convert gyro from degrees/sec (dps) to rad/sec
deg2rad = pi/180;

% Convert accel from g to m/s²
g_const = 9.81;

% Load CSV into a table
T = readtable(csvFile);
T = T(T.d <= 50, :);
% Expecting columns:
%   Distance_in       (inches)
%   Gyro_X_dps, Gyro_Y_dps, Gyro_Z_dps
%   Accel_X_g, Accel_Y_g, Accel_Z_g

distances_in = T.d;                     % inches
gyro_raw_dps  = [T.gx, T.gy, T.gz];     % dps
accel_raw_g   = [T.ax, T.ay, T.az];     % g

N = height(T);
yaw_rad   = zeros(N,1);    % yaw angle (rad)
x_scan    = zeros(N,1);    % scan point in sensor frame (m)
y_scan    = zeros(N,1);
vel       = zeros(N,3);    % velocity in world frame (m/s)
pos       = zeros(N,3);    % position in world frame (m)

% Integrate Gyro_Z to get yaw (heading) over time
for i = 1:N
    omega_z_dps = gyro_raw_dps(i,3);       % deg/s
    omega_z_rad = omega_z_dps * deg2rad;   % rad/s
    if i == 1
        yaw_rad(i) = omega_z_rad * dt;
    else
        yaw_rad(i) = yaw_rad(i-1) + omega_z_rad * dt;
    end
end

% Convert each distance reading into (x,y) in sensor frame
% Distances are in inches -> convert to meters:
dist_m = distances_in * 0.0254;  % 1 in = 0.0254 m
for i = 1:N
    theta = yaw_rad(i);
    x_scan(i) = dist_m(i) * cos(theta);
    y_scan(i) = dist_m(i) * sin(theta);
end

% Integrate accelerometer to obtain sensor’s displacement in the world frame
for i = 1:N
    % 1) Threshold raw accel in g: any |ax|<0.01 or |ay|<0.01 -> set to zero
    ax_g = accel_raw_g(i,1)
    ay_g = accel_raw_g(i,2);
    az_g = accel_raw_g(i,3);

    if abs(ax_g) < 0.03
        ax_g = 0;
    end
    if abs(ay_g) < 0.03
        ay_g = 0;
    end
    a_sens = [ax_g, ay_g, az_g] * g_const  % [ax, ay, az] in m/s2

    % Rotate accel from sensor frame to world frame in XY plane
    Rz = [ cos(yaw_rad(i)), -sin(yaw_rad(i)), 0;
           sin(yaw_rad(i)),  cos(yaw_rad(i)), 0;
           0               ,  0               , 1 ];
    a_world = (Rz * a_sens')';

    if i == 1
        vel(i,:) = a_world * dt;
        pos(i,:) = 0.5 * a_world * dt^2;
    else
        vel(i,:) = vel(i-1,:) + a_world * dt;
        pos(i,:) = pos(i-1,:) + vel(i-1,:) * dt + 0.5 * a_world * dt^2;
    end
end

if(movement)
    X_world = y_scan + pos(:,1);
    Y_world = x_scan + pos(:,2);
else
    X_world = y_scan; % + pos(:,1);
    Y_world = x_scan; % + pos(:,2);
end
close all;
figure;
plot(-X_world, Y_world, '.-');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
title('D + IMU Reconstructed Scan');
if(movement)
    hold on;
    plot(-pos(:,1), pos(:,2), 'r-', 'LineWidth', 2);
    legend('Scan points','Device path','Location','Best');
    Connect each scan point to its device location
    for i = 1:N
        devX = -pos(i,1);
        devY =  pos(i,2);
        scanX = -X_world(i);
        scanY =  Y_world(i);
        plot([devX, scanX], [devY, scanY], 'g-');
    end
end
% 
% hold off;
