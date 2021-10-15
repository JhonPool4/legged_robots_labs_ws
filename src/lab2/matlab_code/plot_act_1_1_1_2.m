% plots - ur5 joint position
% act_1.1 y act_1.2
clc, clear all, close all;

act_dir  = 'act_1.1/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/document/images/', act_dir);

data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = 5000;
      
pos_des= [  data.q1_des(t_start:t_step:t_end), ...
                    data.q2_des(t_start:t_step:t_end), ...
                    data.q3_des(t_start:t_step:t_end), ...
                    data.q4_des(t_start:t_step:t_end), ...
                    data.q5_des(t_start:t_step:t_end), ...
                    data.q6_des(t_start:t_step:t_end)];  
               
vel_des= [   data.dq1_des(t_start:t_step:t_end), ...
                    data.dq2_des(t_start:t_step:t_end), ...
                    data.dq3_des(t_start:t_step:t_end), ...
                    data.dq4_des(t_start:t_step:t_end), ...
                    data.dq5_des(t_start:t_step:t_end), ...
                    data.dq6_des(t_start:t_step:t_end)];  

accel_des= [data.ddq1_des(t_start:t_step:t_end), ...
                    data.ddq2_des(t_start:t_step:t_end), ...
                    data.ddq3_des(t_start:t_step:t_end), ...
                    data.ddq4_des(t_start:t_step:t_end), ...
                    data.ddq5_des(t_start:t_step:t_end), ...
                    data.ddq6_des(t_start:t_step:t_end)];  
 
               
%% joint position
clc, close all;

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{q}$',num2str(i),' ($\mathrm{rad}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), pos_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
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
    screen = get(0, 'ScreenSize');    
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{\dot{q}}$',num2str(i),'  ($\mathrm{\frac{rad}{s}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), vel_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
    
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
clc, close all

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

for i=1:6
    plot_name = strcat('$\mathrm{\ddot{q}}$',num2str(i),'  ($\mathrm{\frac{rad}{s^2}}$)');
    subplot(3, 2, i),
    plot(time(t_start:t_step:t_end), accel_des(:, i), '-k', 'linewidth', 2), hold on, grid on, box on
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

