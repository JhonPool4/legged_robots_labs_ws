% ===============================================================
%	Curso     :   Legged robots
% Alumno  :   Jhon Charaja
% Lab        :   3 (Task space motion control)
% Activity  :   >= 3.2 
% Info        :	position and orientation trajectory tracking
% ===============================================================
clc, close all, clear all;

act_dir  = 'act_3.2/'; % change with the activity name
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

% error: orientation
e_o = [ data.eo_x(t_start:t_step:t_end), ...
            data.eo_y(t_start:t_step:t_end), ...
            data.eo_z(t_start:t_step:t_end)];
norm_ep = [norm(100*p_e(:,1)), norm(100*p_e(:,2)), norm(100*p_e(:,3))]/length(time); % cm
norm_eo = [norm(e_o(:,1)), norm(e_o(:,2)), norm(e_o(:,3))]/length(time); % rad


% angular velocity: desired
w_des = [data.wx_des(t_start:t_step:t_end), ...
                data.wy_des(t_start:t_step:t_end), ...
                data.wz_des(t_start:t_step:t_end)];
% angular velocity: measured
w_med = [data.wx_med(t_start:t_step:t_end), ...
                data.wy_med(t_start:t_step:t_end), ...
                data.wz_med(t_start:t_step:t_end)];
% angular velocity: error
w_e = w_des - w_med;

% angular acceleration: desired
dw_des = [data.dwx_des(t_start:t_step:t_end), ...
                data.dwy_des(t_start:t_step:t_end), ...
                data.dwz_des(t_start:t_step:t_end)];
% angular acceleration: measured
dw_med = [data.dwx_med(t_start:t_step:t_end), ...
                data.dwy_med(t_start:t_step:t_end), ...
                data.dwz_med(t_start:t_step:t_end)];
% angular acceleration: error
dw_e = dw_des - dw_med;


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
    xticks(0:1:5)
    xlim([0 5])
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
    xticks(0:1:5)
    xlim([0 5])
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
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_acceleration');
saveas(gcf,file_name,'epsc')     


%% error: orientation (axis/angle)
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{{e}_{o,x}}$", "$\mathrm{{e}_{o,y}}$", "$\mathrm{{e}_{o,z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
    subplot(3, 1, i),
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
file_name     = fullfile(image_path, 'ee_orientation_error');
saveas(gcf,file_name,'epsc')     

%% error: angular velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{{w}_{e,x}}$", "$\mathrm{{w}_{e,y}}$", "$\mathrm{{w}_{e,z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), w_e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_w_error');
saveas(gcf,file_name,'epsc')     

%% error: angular acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 6.0 20.0])

name_list = ["$\mathrm{\dot{w}_{e,x}}$", "$\mathrm{\dot{w}_{e,y}}$", "$\mathrm{\dot{w}_{e,z}}$"];
for i=1:3 
plot_name = strcat( name_list(i),' ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 1, i),
    plot(time(t_start:t_step:t_end), w_e(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5) 
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
end         

% Save image
file_name     = fullfile(image_path, 'ee_dw_error');
saveas(gcf,file_name,'epsc')   