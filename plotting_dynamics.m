close all;

ref = load('evaluation_dynamics_recurdyn.txt');
data = load('evaluation_dynamics_matlab.txt');

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
    if i == 3
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
    ylabel(sprintf('Position q dot %d [rad/s]', i))
    if i == 3
        legend('Ref','Analysis')
    end
    set(gca,'FontSize',13)
end

figure
for i = 1 : 6
    subplot(2,3,i)
    set(gcf,'Color',[1,1,1])
    plot(ref(:,2), ref(:,i + 14), 'b', 'LineWidth',2.5)
    hold on
    plot(data(:,1), data(:,i + 13), 'r--','LineWidth', 2.5)
    grid on
    xlabel('Time [sec]')
    ylabel(sprintf('Position q ddot %d [rad/s^2]', i))
    if i == 3
        legend('Ref','Analysis')
    end
    set(gca,'FontSize',13)
end

