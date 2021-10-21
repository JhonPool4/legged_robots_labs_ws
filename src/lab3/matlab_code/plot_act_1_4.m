% ===============================================================
%	Curso   :   Legged robots
% 	Alumno  :   Jhon Charaja
%   Lab     :   3 (Task space motion control)
%   Activity:   >= 3.3 
% 	Info	:	trajectory tracking performances
% ===============================================================
clc, close all;

act_dir  = 'act_1.4/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/document/images/', act_dir);

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = length(time); %5000;
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


% angular position: measured
q_med= [    data.q1_med(t_start:t_step:t_end), ...
            data.q2_med(t_start:t_step:t_end), ...
            data.q3_med(t_start:t_step:t_end), ...
            data.q4_med(t_start:t_step:t_end), ...
            data.q5_med(t_start:t_step:t_end), ...
            data.q6_med(t_start:t_step:t_end)];  

% angular velocity: measured               
dq_med= [   data.dq1_med(t_start:t_step:t_end), ...
            data.dq2_med(t_start:t_step:t_end), ...
            data.dq3_med(t_start:t_step:t_end), ...
            data.dq4_med(t_start:t_step:t_end), ...
            data.dq5_med(t_start:t_step:t_end), ...
            data.dq6_med(t_start:t_step:t_end)];  

% angular acceleration: measured
ddq_med= [  data.ddq1_med(t_start:t_step:t_end), ...
            data.ddq2_med(t_start:t_step:t_end), ...
            data.ddq3_med(t_start:t_step:t_end), ...
            data.ddq4_med(t_start:t_step:t_end), ...
            data.ddq5_med(t_start:t_step:t_end), ...
            data.ddq6_med(t_start:t_step:t_end)];  


% cartesian position: measured
p_med = [   data.x_med(t_start:t_step:t_end), ...
            data.y_med(t_start:t_step:t_end), ...
            data.z_med(t_start:t_step:t_end)];  

% cartesian velocity: measured
dp_med = [  data.dx_med(t_start:t_step:t_end), ...
            data.dy_med(t_start:t_step:t_end), ...
            data.dz_med(t_start:t_step:t_end)];        

% cartesian acceleration: measured
ddp_med = [ data.ddx_med(t_start:t_step:t_end), ...
            data.ddy_med(t_start:t_step:t_end), ...
            data.ddz_med(t_start:t_step:t_end)];                  


%% cartesian position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list= ["$\mathrm{x}$", "$\mathrm{y}$", "$\mathrm{z}$"];

for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{m}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), p_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), p_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_position');
saveas(gcf,file_name,'epsc')              

%% cartesian velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{\dot{x}}$", "$\mathrm{\dot{y}}$", "$\mathrm{\dot{z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), dp_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dp_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% add legend
Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.02;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'ee_velocity');
saveas(gcf,file_name,'epsc')              

%% cartesian acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{\ddot{x}}$", "$\mathrm{\ddot{y}}$", "$\mathrm{\ddot{z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s^2}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), ddp_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), ddp_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on

    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_acceleration');
saveas(gcf,file_name,'epsc')     


%% joint position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{q_{1,des}}$", ...
             "$\mathrm{q_{2}}$", ...
             "$\mathrm{q_{3}}$", ...
             "$\mathrm{q_{4}}$", ...
             "$\mathrm{q_{5}}$", ...
             "$\mathrm{q_{6}}$"];

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    %plot(time(t_start:t_step:t_end), q_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), q_med(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
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
             "$\mathrm{\dot{q}_{2}}$", ...
             "$\mathrm{\dot{q}_{3}}$", ...
             "$\mathrm{\dot{q}_{4}}$", ...
             "$\mathrm{\dot{q}_{5}}$", ...
             "$\mathrm{\dot{q}_{6}}$"];

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 2, i),
    %plot(time(t_start:t_step:t_end), dq_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), dq_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
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
             "$\mathrm{\ddot{q}_{2}}$", ...
             "$\mathrm{\ddot{q}_{3}}$", ...
             "$\mathrm{\ddot{q}_{4}}$", ...
             "$\mathrm{\ddot{q}_{5}}$", ...
             "$\mathrm{\ddot{q}_{6}}$"];

for i=1:6
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 2, i),
    %plot(time(t_start:t_step:t_end), ddq_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), ddq_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:0.3:0.9)
    xlim([0 0.9])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'joint_acceleration');
saveas(gcf,file_name,'epsc')     