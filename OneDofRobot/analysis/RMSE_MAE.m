% Script สำหรับคำนวณ Tracking Performance (MAE และ RMSE) พร้อมเซฟรูปกราฟ

% 1. สกัดข้อมูลตัวเลขและเวลาออกจาก Simulink Logging Object
% ใช้ .Values.Data เพื่อดึงค่าตัวเลข และ .Values.Time เพื่อดึงเวลา
% ใช้ squeeze() เพื่อให้แน่ใจว่าข้อมูลออกมาเป็น Column vector แบบ 1 มิติ
q_out = squeeze(out.logsout{6}.Values.Data);
q_ref = squeeze(out.logsout{7}.Values.Data);
time  = out.logsout{6}.Values.Time; % ดึงเวลามาใช้เป็นแกน X

% 2. คำนวณ Tracking Error (e)
tracking_error = q_ref - q_out;

% 3. คำนวณ Mean Absolute Error (MAE)
MAE = mean(abs(tracking_error));

% 4. คำนวณ Root Mean Square Error (RMSE)
RMSE = sqrt(mean(tracking_error.^2));

% 5. แสดงผลลัพธ์บน Command Window
fprintf('=======================================\n');
fprintf('      Tracking Performance Analysis    \n');
fprintf('=======================================\n');
fprintf('Mean Absolute Error (MAE)  : %.6f rad\n', MAE);
fprintf('Root Mean Square Error (RMSE): %.6f rad\n', RMSE);
fprintf('=======================================\n');

% 6. พล็อตกราฟเพื่อนำไปใส่รายงาน (Dark Mode - High Contrast)
fig = figure('Name', 'Tracking Performance', 'Color', [0.15 0.15 0.15], 'Position', [100, 100, 800, 600]);

% --- กราฟบน: เปรียบเทียบตำแหน่ง ---
ax1 = subplot(2, 1, 1);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w'); % พื้นในกราฟสีดำ, แกนและตัวเลขสีขาว
plot(time, q_ref, '--w', 'LineWidth', 1.5); hold on;  % <--- เส้น Reference สีขาวตามที่ขอครับ!
plot(time, q_out, 'c', 'LineWidth', 1.5);             % เส้น Actual สีฟ้าสว่าง (Cyan)
title('Position Tracking: Reference vs Actual', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Position (rad)', 'Color', 'w', 'FontSize', 11);

% ตั้งค่า Legend ให้พื้นดำ ตัวหนังสือขาว
lgd1 = legend('Reference (q_{ref})', 'Actual (q_{out})', 'Location', 'best');
set(lgd1, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', 'w'); 
grid on;
ax1.GridColor = [0.6 0.6 0.6]; % เส้น Grid สีเทา
ax1.GridAlpha = 0.5;

% --- กราฟล่าง: แสดง Error และเส้นขอบเขต MAE / RMSE ---
ax2 = subplot(2, 1, 2);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
plot(time, tracking_error, 'r', 'LineWidth', 1.5); hold on;

% ตีเส้นขอบเขตของค่าเฉลี่ยความผิดพลาด (ใช้สีเขียวสว่าง และสีชมพูสว่าง)
yline(MAE, '--g', 'MAE Bound', 'Color', 'g', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(-MAE, '--g', 'Color', 'g', 'LineWidth', 1.5);
yline(RMSE, '--m', 'RMSE Bound', 'Color', 'm', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(-RMSE, '--m', 'Color', 'm', 'LineWidth', 1.5);

title('Tracking Error Profile', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)', 'Color', 'w', 'FontSize', 11);
ylabel('Error (rad)', 'Color', 'w', 'FontSize', 11);

lgd2 = legend('Tracking Error (e)', 'MAE', 'RMSE', 'Location', 'best');
set(lgd2, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', 'w');
grid on;
ax2.GridColor = [0.6 0.6 0.6];
ax2.GridAlpha = 0.5;

% 7. บันทึกกราฟออกเป็นไฟล์รูปภาพ (Auto-Save)
filename = 'Tracking_Performance_Dark.png';
try
    exportgraphics(fig, filename, 'Resolution', 300);
    fprintf('>> สำเร็จ! บันทึกรูปภาพกราฟไว้ที่ไฟล์: %s\n', filename);
catch
    saveas(fig, filename);
    fprintf('>> สำเร็จ! บันทึกรูปภาพกราฟไว้ที่ไฟล์: %s\n', filename);
end