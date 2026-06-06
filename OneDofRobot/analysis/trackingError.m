% =========================================================================
% MATLAB Script สำหรับ Lab 3: Tracking Error & Trajectory Profile Analysis
% =========================================================================

%% 1. สกัดข้อมูลจาก Simulink (แก้ตัวเลข Index ให้ตรงกับบล็อกใน Simulink ของคุณ)
time    = out.logsout{1}.Values.Time;               % ดึงแกนเวลา
q_ref   = squeeze(out.logsout{7}.Values.Data);      % Reference Position
qd_ref  = squeeze(out.logsout{8}.Values.Data);      % Reference Velocity
qdd_ref = squeeze(out.logsout{11}.Values.Data);      % Reference Acceleration
j_out   = squeeze(out.logsout{5}.Values.Data);      % Actual Jerk (หรือ Reference Jerk)
q_out   = squeeze(out.logsout{6}.Values.Data);      % Actual Position

%% 2. คำนวณ Tracking Performance
tracking_error = q_ref - q_out;
MAE = mean(abs(tracking_error));
RMSE = sqrt(mean(tracking_error.^2));

% พิมพ์ผลลัพธ์ลง Command Window
fprintf('=======================================\n');
fprintf('   Lab 3: Trajectory & Tracking Stats  \n');
fprintf('=======================================\n');
fprintf('Mean Absolute Error (MAE)  : %.6f rad\n', MAE);
fprintf('Root Mean Square Error (RMSE): %.6f rad\n', RMSE);
fprintf('Max Absolute Jerk          : %.4f rad/s^3\n', max(abs(j_out)));
fprintf('=======================================\n');

%% 3. พล็อตกราฟ 1: Tracking Error Analysis (Dark Mode)
fig1 = figure('Name', '1. Tracking Performance', 'Color', [0.15 0.15 0.15], 'Position', [100, 100, 800, 600]);

% กราฟบน: Position
ax1 = subplot(2, 1, 1);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, q_ref, '--w', 'LineWidth', 1.5); hold on;
plot(time, q_out, 'c', 'LineWidth', 1.5);
title('Position Tracking: Reference vs Actual', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Position (rad)', 'Color', 'w');
lgd1 = legend('Reference', 'Actual', 'Location', 'best');
set(lgd1, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', 'w');
grid on; ax1.GridColor = [0.6 0.6 0.6]; ax1.GridAlpha = 0.5;

% กราฟล่าง: Error
ax2 = subplot(2, 1, 2);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, tracking_error, 'r', 'LineWidth', 1.5); hold on;
yline(MAE, '--g', 'MAE', 'Color', 'g', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(-MAE, '--g', 'Color', 'g', 'LineWidth', 1.5);
yline(RMSE, '--m', 'RMSE', 'Color', 'm', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(-RMSE, '--m', 'Color', 'm', 'LineWidth', 1.5);
title('Tracking Error Profile', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'Color', 'w'); ylabel('Error (rad)', 'Color', 'w');
lgd2 = legend('Tracking Error', 'MAE', 'RMSE', 'Location', 'best');
set(lgd2, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', 'w');
grid on; ax2.GridColor = [0.6 0.6 0.6]; ax2.GridAlpha = 0.5;

%% 4. พล็อตกราฟ 2: Trajectory Kinematics Profile (Dark Mode)
fig2 = figure('Name', '2. Trajectory Profiles (Pos, Vel, Acc, Jerk)', 'Color', [0.15 0.15 0.15], 'Position', [920, 100, 800, 800]);

% 1. Position Profile
ax3 = subplot(4, 1, 1); set(ax3, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, q_ref, 'c', 'LineWidth', 1.5);
title('Kinematics Profile Analysis', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Pos (rad)', 'Color', 'w'); grid on; ax3.GridColor = [0.6 0.6 0.6]; ax3.GridAlpha = 0.5;

% 2. Velocity Profile
ax4 = subplot(4, 1, 2); set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, qd_ref, 'g', 'LineWidth', 1.5);
ylabel('Vel (rad/s)', 'Color', 'w'); grid on; ax4.GridColor = [0.6 0.6 0.6]; ax4.GridAlpha = 0.5;

% 3. Acceleration Profile
ax5 = subplot(4, 1, 3); set(ax5, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, qdd_ref, 'y', 'LineWidth', 1.5);
ylabel('Acc (rad/s^2)', 'Color', 'w'); grid on; ax5.GridColor = [0.6 0.6 0.6]; ax5.GridAlpha = 0.5;

% 4. Jerk Profile
ax6 = subplot(4, 1, 4); set(ax6, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, j_out, 'm', 'LineWidth', 1.2);
ylabel('Jerk (rad/s^3)', 'Color', 'w'); xlabel('Time (s)', 'Color', 'w');
grid on; ax6.GridColor = [0.6 0.6 0.6]; ax6.GridAlpha = 0.5;

%% 5. Auto-Save รูปภาพ
% ตั้งชื่อไฟล์ให้สัมพันธ์กับโพรไฟล์ที่ทดสอบ เช่น 'Lab3_Trapezoidal_Error.png'
profile_name = 'Trapezoidal'; % <--- เปลี่ยนตรงนี้เป็น 'SCurve' หรือ 'Quintic' ตอนเก็บผล

try
    exportgraphics(fig1, ['Lab3_', profile_name, '_Error.png'], 'Resolution', 300);
    exportgraphics(fig2, ['Lab3_', profile_name, '_Profile.png'], 'Resolution', 300);
    disp(['>> บันทึกรูปภาพ ', profile_name, ' สำเร็จ!']);
catch
    saveas(fig1, ['Lab3_', profile_name, '_Error.png']);
    saveas(fig2, ['Lab3_', profile_name, '_Profile.png']);
    disp(['>> บันทึกรูปภาพ ', profile_name, ' สำเร็จ!']);
end