% ===============================================================
%	Curso     :   Legged robots
% Alumno  :   Jhon Charaja
% Lab        :   3 (Task space motion control)
% Activity  :   3.1 
% Info        :	trajectory tracking error
% ===============================================================
clc, close all, clear all;

act_dir  = 'act_3.6/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab3/document/images/', act_dir);

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = length(time); %5000;


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

% error: cartesian position
p_e = p_des - p_med;

% error: cartesian velocity
dp_e = dp_des - dp_med;

% error: cartesian acceleration
ddp_e = ddp_des - ddp_med;

% error: orientation angle/axis
e_o = [ data.eo_x(t_start:t_step:t_end), ...
            data.eo_y(t_start:t_step:t_end), ...
            data.eo_z(t_start:t_step:t_end)];
% error: orientation rpy
e_rpy = [ data.r_e(t_start:t_step:t_end), ...
            data.p_e(t_start:t_step:t_end), ...
            data.y_e(t_start:t_step:t_end)]; 

% orientation rpy: measured
rpy_med = [ data.r_m(t_start:t_step:t_end), ...
            data.p_m(t_start:t_step:t_end), ...
            data.y_m(t_start:t_step:t_end)]; 
% orientation rpy: desired
rpy_des = [ data.r_d(t_start:t_step:t_end), ...
            data.p_d(t_start:t_step:t_end), ...
            data.y_d(t_start:t_step:t_end)]; 

norm_ep = [norm(100*p_e(:,1)), norm(100*p_e(:,2)), norm(100*p_e(:,3))]/length(time); % cm
norm_eo = [norm(e_o(:,1)), norm(e_o(:,2)), norm(e_o(:,3))]/length(time); % rad
norm_erpy = [norm(e_rpy(:,1)), norm(e_rpy(:,2)), norm(e_rpy(:,3))]/length(time); % rad
%% error: cartesian position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list= ["$\mathrm{e_x}$", "$\mathrm{e_y}$", "$\mathrm{e_z}$"];

for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{m}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), p_e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_position_error');
saveas(gcf,file_name,'epsc')      

%% error: cartesian velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{\dot{e}_x}$", "$\mathrm{\dot{e}_y}$", "$\mathrm{\dot{e}_z}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), dp_e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_velocity_error');
saveas(gcf,file_name,'epsc')     

%% error: cartesian acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{\ddot{e}_x}$", "$\mathrm{\ddot{e}_y}$", "$\mathrm{\ddot{e}_z}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s^2}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), ddp_e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_acceleration_error');
saveas(gcf,file_name,'epsc')     

%% error: orientation (axis/angle)
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 20.0 5.0])

name_list = ["$\mathrm{{e}_{o,x}}$", "$\mathrm{{e}_{o,y}}$", "$\mathrm{{e}_{o,z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{rad}$)');
    subplot(1, 3, i),
    plot(time(t_start:t_step:t_end), e_o(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_orientation_error_angle_axis');
saveas(gcf,file_name,'epsc')     

%% error: orientation (rpy)
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 20.0 5.0])

name_list = ["$\mathrm{{\alpha}_{e}}$", "$\mathrm{{\beta}_{e}}$", "$\mathrm{{\gamma}_{e}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{rad}$)');
    subplot(1, 3, i),
    plot(time(t_start:t_step:t_end), e_rpy(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_orientation_error_rpy');
saveas(gcf,file_name,'epsc')     

%% error: orientation (rpy)
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 20.0 6])

name_list = ["$\mathrm{{\alpha}}$", "$\mathrm{{\beta}}$", "$\mathrm{{\gamma}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{rad}$)');
    subplot(1, 3, i),
    plot(time(t_start:t_step:t_end), rpy_des(:, i), '-r', 'linewidth', 2), hold on, grid on, box on
    plot(time(t_start:t_step:t_end), rpy_med(:, i), '--k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         
% add legend
%Lgnd = legend({'desired', 'measured'}, 'interpreter', 'latex', 'Orientation','horizontal');
%Lgnd.FontSize = 12;
%Lgnd.Position(1) = 0.35;
%Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'ee_orientation_tracking_rpy');
saveas(gcf,file_name,'epsc')    