% ===============================================================
%	Curso     :   Legged robots
% Alumno  :   Jhon Charaja
% Lab        :   3 (Task space motion control)
% Activity  :   >= 3.1 
% Info        :	trajectory tracking error
% ===============================================================
clc, close all, clear all;

act_dir  = 'act_3.1/'; % change with the activity name
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

% error: cartesian position
p_e = p_des - p_med;
norm_ep = [norm(100*p_e(:,1)), norm(100*p_e(:,2)), norm(100*p_e(:,3))]/length(time); % cm
norm_eo = [norm(e_o(:,1)), norm(e_o(:,2)), norm(e_o(:,3))]/length(time); % rad

% error: cartesian velocity
dp_e = dp_des - dp_med;

% error: cartesian acceleration
ddp_e = ddp_des - ddp_med;

% error: orientation
e_o = [ data.eo_x(t_start:t_step:t_end), ...
            data.eo_y(t_start:t_step:t_end), ...
            data.eo_z(t_start:t_step:t_end)];
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
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
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
    set(gcf,'units','centimeters','position', [0 0 20.0 6.0])

name_list = ["$\mathrm{{e}_{o,x}}$", "$\mathrm{{e}_{o,y}}$", "$\mathrm{{e}_{o,z}}$"];
for i=1:3
    plot_name = strcat( name_list(i),' ($\mathrm{\frac{m}{s}}$)');
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
file_name     = fullfile(image_path, 'ee_orientation_error');
saveas(gcf,file_name,'epsc')     
