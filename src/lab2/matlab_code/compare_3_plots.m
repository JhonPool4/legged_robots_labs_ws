% plots - ur5 joint position

clc, clear all, close all;

% colors
color1 = [0, 0.4470, 0.7410];               % black         --alpha=0.1 
color2 = [0.6350, 0.0780, 0.1840];       % dark blue     --alpha=0.5
color3 = [0.4660, 0.6740, 0.1880];      % dark green    --alpha=0.8

% kp 60
act_dir  = 'act_2.4_kp_60/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/document/images/', act_dir);
data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

time = data.t;

t_start = 1;
t_step  = 1;
t_end   = 5000;

e_60 = [data.e1(t_start:t_step:t_end), ...
        data.e2(t_start:t_step:t_end), ...
        data.e3(t_start:t_step:t_end), ...
        data.e4(t_start:t_step:t_end), ...
        data.e5(t_start:t_step:t_end), ...
        data.e6(t_start:t_step:t_end)
        ];
% kp 300
act_dir  = 'act_2.4_kp_300/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/document/images/', act_dir);
data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

e_300 = [data.e1(t_start:t_step:t_end), ...
        data.e2(t_start:t_step:t_end), ...
        data.e3(t_start:t_step:t_end), ...
        data.e4(t_start:t_step:t_end), ...
        data.e5(t_start:t_step:t_end), ...
        data.e6(t_start:t_step:t_end)
        ];  

% kp 600
act_dir  = 'act_2.4_kp_600/'; % change with the activity name
file_name = 'data';
file_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/data/', act_dir);
image_path = fullfile('/home/jhon/catkin_ws/labs_ws/src/lab1/document/images/', act_dir);
data = readtable(fullfile(file_path, file_name),'PreserveVariableNames',true);

e_600 = [data.e1(t_start:t_step:t_end), ...
        data.e2(t_start:t_step:t_end), ...
        data.e3(t_start:t_step:t_end), ...
        data.e4(t_start:t_step:t_end), ...
        data.e5(t_start:t_step:t_end), ...
        data.e6(t_start:t_step:t_end)
        ];  

figure(1), hold on, grid on, box on;
    set(gcf,'units','centimeters','position', [0 0 15.0 20.0])

name_list = ["$\mathrm{e_1}$", "$\mathrm{e_2}$","$\mathrm{e_3}$","$\mathrm{e_4}$","$\mathrm{e_5}$","$\mathrm{e_6}$"];

for i=1:6
    plot_name = strcat(name_list(i),'  ($\mathrm{{rad}}$)');
    subplot(3, 2, i),
    p1=plot(time(t_start:t_step:t_end),  e_60(:, i), 'color', color1, 'linestyle', '--', 'linewidth', 1), hold on, grid on, box on
    p2=plot(time(t_start:t_step:t_end), e_300(:, i),'color', color2, 'linestyle', '--', 'linewidth', 1), hold on, grid on, box on
    p3=plot(time(t_start:t_step:t_end), e_600(:, i),'color', color3, 'linestyle', '--', 'linewidth', 1), hold on, grid on, box on
    xlabel('time (s)', 'interpreter', 'latex')
    ylabel(plot_name, 'interpreter', 'latex')
    xticks(0:1:5)
    xlim([0 5])
    ax = gca; % current axes
    ax.FontSize = 12;
    set(gca,'TickLabelInterpreter','latex')

    %p1.MarkerIndices = 1:900:length(e_60(:,i));
    %p2.MarkerIndices = 300:900:length(e_300(:,i));   
    %p3.MarkerIndices = 600:900:length(e_600(:,i));
    
    %p1.MarkerFaceColor = color1;
    %p2.MarkerFaceColor = color2;
    %p3.MarkerFaceColor = color3;

    %p1.MarkerSize = 8;
    %p2.MarkerSize = 8;
    %p3.MarkerSize = 8;    
end     
% add legend
Lgnd = legend({'$k_p=60$', '$k_p=300$', '$k_p=600$'}, 'interpreter', 'latex', 'Orientation','horizontal');
Lgnd.FontSize = 12;
Lgnd.Position(1) = 0.24;
Lgnd.Position(2) = 0.95;

% Save image
file_name     = fullfile(image_path, 'e_kps');
saveas(gcf,file_name,'epsc')  