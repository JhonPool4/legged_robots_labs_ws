% ===============================================================
%	Curso   :   Legged robots
% 	Alumno  :   Jhon Charaja
%   Lab     :   3 (Task space motion control)
%   Activity:   3.1 and 3.2
% 	Info	:	sinusoidal and reference in x axis
% ===============================================================
clc, clear all, close all;

act_dir  = 'act_3.1/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/document/images/', act_dir);

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = 5000;
% angular position: desired
q_des= [    data.q1_des(t_start:t_step:t_end), ...
            data.q2_des(t_start:t_step:t_end), ...
            data.q3_des(t_start:t_step:t_end), ...
            data.q4_des(t_start:t_step:t_end), ...
            data.q5_des(t_start:t_step:t_end), ...
            data.q6_des(t_start:t_step:t_end)];  

% angular velocity: desired               
dq_des= [   data.dq1_des(t_start:t_step:t_end), ...
            data.dq2_des(t_start:t_step:t_end), ...
            data.dq3_des(t_start:t_step:t_end), ...
            data.dq4_des(t_start:t_step:t_end), ...
            data.dq5_des(t_start:t_step:t_end), ...
            data.dq6_des(t_start:t_step:t_end)];  

% angular acceleration: desired
ddq_des= [  data.ddq1_des(t_start:t_step:t_end), ...
            data.ddq2_des(t_start:t_step:t_end), ...
            data.ddq3_des(t_start:t_step:t_end), ...
            data.ddq4_des(t_start:t_step:t_end), ...
            data.ddq5_des(t_start:t_step:t_end), ...
            data.ddq6_des(t_start:t_step:t_end)];  


% cartesian position: desired
p_des = [   data.x_des(t_start:t_step:t_end), ...
            data.y_des(t_start:t_step:t_end), ...
            data.z_des(t_start:t_step:t_end)];  

% cartesian velocity: desired
dp_des = [  data.dx_des(t_start:t_step:t_end), ...
            data.dy_des(t_start:t_step:t_end), ...
            data.dz_des(t_start:t_step:t_end)];        

% cartesian acceleration: desired
ddp_des = [ data.ddx_des(t_start:t_step:t_end), ...
            data.ddy_des(t_start:t_step:t_end), ...
            data.ddz_des(t_start:t_step:t_end)];                  


%% cartesian position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 10.0 20.0])

name_list = ["$\mathrm{x_{des}}$", "$\mathrm{y_{des}}$", "$\mathrm{z_{des}}$"]
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{m}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), p_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'cartesian_position');
saveas(gcf,file_name,'epsc')              

%% cartesian velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 10.0 20.0])

name_list = ["$\mathrm{\dot{x}_{des}}$", "$\mathrm{\dot{y}_{des}}$", "$\mathrm{\dot{z}_{des}}$"]
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), dp_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'cartesian_velocity');
saveas(gcf,file_name,'epsc')              

%% cartesian acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 10.0 20.0])

name_list = ["$\mathrm{\ddot{x}_{des}}$", "$\mathrm{\ddot{y}_{des}}$", "$\mathrm{\ddot{z}_{des}}$"]
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s^2}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), ddp_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'cartesian_acceleration');
saveas(gcf,file_name,'epsc')     


%% joint position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{q_{1,des}}$", ...
             "$\mathrm{q_{2,des}}$", ...
             "$\mathrm{q_{3,des}}$", ...
             "$\mathrm{q_{4,des}}$", ...
             "$\mathrm{q_{5,des}}$", ...
             "$\mathrm{q_{6,des}}$"]

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), q_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'joint_position');
saveas(gcf,file_name,'epsc')     


%% joint velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{\dot{q}_{1,des}}$", ...
             "$\mathrm{\dot{q}_{2,des}}$", ...
             "$\mathrm{\dot{q}_{3,des}}$", ...
             "$\mathrm{\dot{q}_{4,des}}$", ...
             "$\mathrm{\dot{q}_{5,des}}$", ...
             "$\mathrm{\dot{q}_{6,des}}$"]

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), dq_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'joint_velocity');
saveas(gcf,file_name,'epsc')     


%% joint acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{\ddot{q}_{1,des}}$", ...
             "$\mathrm{\ddot{q}_{2,des}}$", ...
             "$\mathrm{\ddot{q}_{3,des}}$", ...
             "$\mathrm{\ddot{q}_{4,des}}$", ...
             "$\mathrm{\ddot{q}_{5,des}}$", ...
             "$\mathrm{\ddot{q}_{6,des}}$"]

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), ddq_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'joint_acceleration');
saveas(gcf,file_name,'epsc')     