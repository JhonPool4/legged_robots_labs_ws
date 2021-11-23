% ===============================================================
%	Curso   :   Legged robots
% Alumno  :   Jhon Charaja
% Lab     :   4 (Impedance control)
% Activity:   >= 1.1 
% Info	:	joint configuration (q, dq, ddq) vs external torque (tau)
% ===============================================================
clc, close all, clear all;

act_dir  = 'act_2.1/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab4/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab4/document/images/', act_dir);

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

% time
time = data.t;
t_start = 3000;
t_step  = 1;
t_end   = length(time); 

% external torque
tau_ext= [    data.tau_ext_1(t_start:t_step:t_end), ...
            data.tau_ext_2(t_start:t_step:t_end), ...
            data.tau_ext_3(t_start:t_step:t_end), ...
            data.tau_ext_4(t_start:t_step:t_end), ...
            data.tau_ext_5(t_start:t_step:t_end), ...
            data.tau_ext_6(t_start:t_step:t_end)];  
% joint configuration
[q_med, dq_med, ddq_med, q_des, dq_des, ddq_des] = get_articular_data(data,t_start, t_step, t_end);


% colors
color1 = [0, 0.4470, 0.7410];                % dark red        --alpha=0.1 
color2 = [0.6350, 0.0780, 0.1840];       % dark blue      --alpha=0.5
color3 = [0.4660, 0.6740, 0.1880];       % dark green    --alpha=0.8

img_height = 22.0; % cm
img_width = 16.0; % cm

cnt_img_h = 2;
cnt_img_v = 3;
space_size = 1;
img_size = 4;
legend_flag = true;

%% external torque vs joint position 
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 img_width img_height])

name_list_x = ["$\mathrm{q_1}$", "$\mathrm{q_2}$","$\mathrm{q_3}$","$\mathrm{q_4}$","$\mathrm{q_5}$","$\mathrm{q_6}$"];
name_list_y = ["$\mathrm{\tau_{ext}}$"];    

for img_index=1:6
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{rad}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N.m}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(q_med(:,img_index), tau_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_torque_vs_joint_position');
saveas(gcf,file_name,'epsc')  

%% external torque vs joint velocity
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 16.0 20.0])

name_list_x = [   "$\mathrm{\dot{q}_1}$", ...
                        "$\mathrm{\dot{q}_2}$",...
                        "$\mathrm{\dot{q}_3}$",...
                        "$\mathrm{\dot{q}_4}$",...
                        "$\mathrm{\dot{q}_5}$",...
                        "$\mathrm{\dot{q}_6}$"];

for img_index=1:6
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{\frac{rad}{s}}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N.m}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(dq_med(:,img_index), tau_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_torque_vs_joint_velocity');
saveas(gcf,file_name,'epsc')  


%% external torque vs joint acceleration
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 16.0 20.0])

name_list_x = [   "$\mathrm{\ddot{q}_1}$", ...
                        "$\mathrm{\ddot{q}_2}$",...
                        "$\mathrm{\ddot{q}_3}$",...
                        "$\mathrm{\ddot{q}_4}$",...
                        "$\mathrm{\ddot{q}_5}$",...
                        "$\mathrm{\ddot{q}_6}$"];


for img_index=1:6
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{\frac{rad}{s^2}}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N.m}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(ddq_med(:,img_index), tau_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_torque_vs_joint_acceleration');
saveas(gcf,file_name,'epsc')  