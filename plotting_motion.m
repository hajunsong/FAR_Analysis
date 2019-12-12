close all;

ref = load('evaluation_motion_path_generator_recurdyn.txt');
data = load('evaluation_motion_path_generator_matlab.txt');
% data = load('evaluation_vsd.txt');

% ref = load('evaluation_motion_recurdyn.txt');
% data = load('evaluation_motion_matlab.txt');

ylabel_txt = {'End X [m]','End Y [m]','End Z [m]','End Roll [rad]','End Pitch [rad]','End Yaw [rad]'};
ylabel_dot_txt = {'VX [m]','VY [m]','VZ [m]','WX [rad]','WY [rad]','WZ [rad]'};

figure
for i = 1 : 6
    subplot(2,3,i)
    set(gcf,'Color',[1,1,1])
    plot(ref(:,2), ref(:,i + 2), 'b', 'LineWidth',2.5)
    hold on
    plot(data(:,1), data(:,i + 1), 'r--','LineWidth', 2.5)
    grid on
    xlabel('Time [sec]')
    ylabel(sprintf('Position q %d [rad]', i))
    if i == 6
        legend('Ref','Analysis')
    end
    set(gca,'FontSize',13)
end

figure
for i = 1 : 6
    subplot(2,3,i)
    set(gcf,'Color',[1,1,1])
    plot(ref(:,2), ref(:,i + 8), 'b', 'LineWidth',2.5)
    hold on
    plot(data(:,1), data(:,i + 7), 'r--','LineWidth', 2.5)
    grid on
    xlabel('Time [sec]')
    ylabel(ylabel_txt{i})
    if i == 3
        legend('Ref','Analysis')
    end
    set(gca,'FontSize',13)
end
% 
% figure
% for i = 1 : 6
%     subplot(4,6,i+12)
%     set(gcf,'Color',[1,1,1])
%     plot(ref(:,2), ref(:,i + 14), 'b', 'LineWidth',2.5)
%     hold on
%     plot(data(:,1), data(:,i + 13), 'r--','LineWidth', 2.5)
%     grid on
%     xlabel('Time [sec]')
%     ylabel(sprintf('Velocity q dot %d [rad]', i))
%     if i == 3
% %         legend('Ref','Analysis')
%     end
%     set(gca,'FontSize',13)
% end
% 
% % figure
% for i = 1 : 6
%     subplot(4,6,i+18)
%     set(gcf,'Color',[1,1,1])
%     plot(ref(:,2), ref(:,i + 20), 'b', 'LineWidth',2.5)
%     hold on
%     plot(data(:,1), data(:,i + 19), 'r--','LineWidth', 2.5)
%     grid on
%     xlabel('Time [sec]')
%     ylabel(ylabel_dot_txt{i})
%     if i == 3
% %         legend('Ref','Analysis')
%     end
%     set(gca,'FontSize',13)
% end

% figure
% set(gcf,'Color',[1,1,1])
% plot(data(:,1), data(:,end), 'o')
% grid on
% xlabel('Time [sec]')
% ylabel('NR count')
% set(gca,'FontSize',13)

