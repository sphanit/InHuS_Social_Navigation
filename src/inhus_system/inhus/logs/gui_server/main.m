clear all;
close all;

data = readtable('data_254.0_280.0.csv');
time = (data{1,:})';
vel_h = (data{2,:})';
vel_r = (data{3,:})';

% yyaxis left;
plot(time-time(1),vel_h,'b','LineWidth',2);
hold on;
plot(time-time(1),vel_r,'r','LineWidth',2);
% yticks(0 : 0.5: 4);
% ylim([0 4]);
% yyaxis right;
% plot(dual_idx,d(dual_idx),'b.','LineWidth',2);
% plot(single_idx,d(single_idx),'k.','LineWidth',2);
% plot(vel_idx,d(vel_idx),'r.','LineWidth',2);
% plot(back_idx,d(back_idx),'g.','LineWidth',2);
% ylim([-5 10]);

legend('Human', 'Robot')
legend boxoff;
hold off;

ylabel('Speeds (m/s)');
xlabel('Time (s)');
ax = gca;
set(gca, 'box', 'off')
ax.FontSize = 12;
ax.FontName = 'Times';
% axis equal;

