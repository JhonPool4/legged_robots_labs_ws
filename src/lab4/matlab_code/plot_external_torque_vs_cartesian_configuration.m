% ===============================================================
%	Curso   :   Legged robots
% Alumno  :   Jhon Charaja
% Lab     :   4 (Impedance control)
% Activity:   >= 2.1 
% Info	:	cartesian configuration (x, dx, ddx) vs external force (f_ext)
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
f_ext= [    data.f_ext_x(t_start:t_step:t_end), ...
            data.f_ext_y(t_start:t_step:t_end), ...
            data.f_ext_z(t_start:t_step:t_end)];

% cartesian configuration
x_med = [data.x_med(t_start:t_step: t_end),...
                data.y_med(t_start:t_step: t_end),...
                data.z_med(t_start:t_step: t_end)];

dx_med = [data.dx_med(t_start:t_step: t_end),...
                data.dy_med(t_start:t_step: t_end),...
                data.dz_med(t_start:t_step: t_end)];

ddx_med = [data.ddx_med(t_start:t_step: t_end),...
                data.ddy_med(t_start:t_step: t_end),...
                data.ddz_med(t_start:t_step: t_end)];

% colors
color1 = [0, 0.4470, 0.7410];                % dark red        --alpha=0.1 
color2 = [0.6350, 0.0780, 0.1840];       % dark blue      --alpha=0.5
color3 = [0.4660, 0.6740, 0.1880];       % dark green    --alpha=0.8

img_height = 22.0; % cm
img_width = 6.5; % cm

cnt_img_h = 1;
cnt_img_v = 3;
space_size = 1;
img_size = 5;
legend_flag = false;


%% external force vs cartesian position 
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 img_width img_height])

name_list_x = ["$\mathrm{x}$", "$\mathrm{y}$","$\mathrm{z}$"];
name_list_y = ["$\mathrm{f_{ext}}$"];    

for img_index=1:3
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{m}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(x_med(:,img_index), f_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_force_vs_cartesian_position');
saveas(gcf,file_name,'epsc')  

%% external force vs cartesian velocity 
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 img_width img_height])

name_list_x = ["$\mathrm{\dot{x}}$", "$\mathrm{\dot{y}}$","$\mathrm{\dot{z}}$"];
name_list_y = ["$\mathrm{f_{ext}}$"];    

for img_index=1:3
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{\frac{m}{s}}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(dx_med(:,img_index), f_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_force_vs_cartesian_velocity');
saveas(gcf,file_name,'epsc')  

%% external force vs cartesian acceleration 
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 img_width img_height])

name_list_x = ["$\mathrm{\ddot{x}}$", "$\mathrm{\ddot{y}}$","$\mathrm{\ddot{z}}$"];
name_list_y = ["$\mathrm{f_{ext}}$"];    

for img_index=1:3
    plot_name_x = strcat(name_list_x(img_index),' ($\mathrm{\frac{m}{s^2}}$)');
    plot_name_y = strcat(name_list_y, ' ($\mathrm{N}$)');
    [rows, cols, list_index] = get_subplot_index(cnt_img_h, cnt_img_v, img_index, img_size, space_size, legend_flag);
    subplot(rows, cols, list_index),
    plot(ddx_med(:,img_index), f_ext(:, img_index), 'linestyle', '-', 'linewidth', 2, 'color', color1), hold on, grid on, box on
    xlabel(plot_name_x, 'interpreter', 'latex')
    ylabel(plot_name_y, 'interpreter', 'latex')
    %xticks(time(t_start):time(t_end))
    %xlim([0 time(t_end)])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')
    
end         

% Save image
file_name     = fullfile(image_path, 'external_force_vs_cartesian_acceleration');
saveas(gcf,file_name,'epsc')  




