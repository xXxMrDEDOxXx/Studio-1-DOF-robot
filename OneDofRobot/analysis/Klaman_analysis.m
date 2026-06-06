% lab3_save_run('lab3_disturb.mat')
% load lab3_baseline.mat
% idx = t >= t(end)-1.5;                       % ช่วงแขนนิ่ง
% nr = std(diff(qd_fd)) / std(diff(qd_out));   % noise ลดกี่เท่า (ควร >>1)
% rms = sqrt(mean((qd_out-qd_fd).^2));         % KF ตามแนวโน้มไหม
% plot(t,qd_fd,'Color',[.7 .7 .7]); hold on; plot(t,qd_out,'c','LineWidth',1.5);
% legend('finite-diff (ดิบ)','Kalman'); xlabel('t (s)'); ylabel('q̇ (rad/s)');
% title(sprintf('noise ลด %.1f เท่า | RMSE=%.3f',nr,rms));

% Ke=0.1045; N=2; R=2.1142;
% i_model = (Vin - Ke*N*qd_out)/R;
% 
% plot(t, i_est,  'y', 'LineWidth', 2);   hold on;   % KF เหลือง หนา (พล็อตก่อน)
% plot(t, i_model,'c--','LineWidth', 1.2);           % model ฟ้าประ ทับบนสุด → เห็นรอยประ
% legend('i_{KF}','i_{model}'); ylabel('current (A)'); xlabel('t (s)');
% title(sprintf('KF current vs model | RMSE=%.3f A', sqrt(mean((i_est-i_model).^2))));
% grid on;

% %% ===== เฟส 3: Q sweep =====
% files = {'lab3_Q_1e-6.mat','lab3_Q_1e-4.mat','lab3_Q_1e-2.mat'};
% Qvals = [1e-6, 1e-4, 1e-2];
% cols  = {'c','y','m'};
% 
% figure('Color','k'); axes('Color','k','XColor','w','YColor','w'); hold on;
% fprintf('Q_VEL\t std(qd) นิ่ง\t lag(ms)\n');
% for k = 1:3
%     S = load(files{k});                       % S.t, S.qd_out, S.qd_ref
%     Fs  = 1/median(diff(S.t));
%     idx = S.t >= S.t(end)-1.5;                % ช่วงแขนนิ่ง → วัด noise
%     sd = std(diff(S.qd_out));                 % smoothness (ต่ำ=เนียน)
%     lag = finddelay(S.qd_ref, S.qd_out)*1000/Fs;  % responsiveness (ต่ำ=ไว)
% 
%     fprintf('%.0e\t %.4f\t %.1f\n', Qvals(k), sd, lag);
%     plot(S.t, S.qd_out, cols{k}, 'LineWidth',1.3, ...
%         'DisplayName', sprintf('Q=%.0e | std=%.3f lag=%.0fms', Qvals(k), sd, lag));
% end
% plot(S.t, S.qd_ref, 'w--', 'LineWidth',1, 'DisplayName','q̇ ref');
% legend('TextColor','w','Color','k'); grid on;
% xlabel('t (s)'); ylabel('q̇ (rad/s)');
% title('Q sweep: responsiveness vs smoothness','Color','w');
% 
% ===== เฟส 3 (4b): R sweep =====
files = {'lab3_R_low.mat','lab3_R_mid.mat','lab3_R_high.mat'};
Rvals = [1.471e-8, 1.471e-7, 1.471e-6];
cols  = {'c','y','m'};

figure('Color','k'); axes('Color','k','XColor','w','YColor','w'); hold on;
fprintf('R_meas\t std(qd) นิ่ง\t lag(ms)\n');
for k = 1:3
    S = load(files{k});                       % S.t, S.qd_out, S.qd_ref
    Fs  = 1/median(diff(S.t));
    idx = S.t >= S.t(end)-1.5;                % ช่วงแขนนิ่ง → วัด noise
    sd = std(diff(S.qd_out));                % smoothness (ต่ำ=เนียน)
    lag = finddelay(S.qd_ref, S.qd_out)*1000/Fs;  % responsiveness (ต่ำ=ไว)

    fprintf('%.3e\t %.4f\t %.1f\n', Rvals(k), sd, lag);
    plot(S.t, S.qd_out, cols{k}, 'LineWidth',1.3, ...
        'DisplayName', sprintf('R=%.2e | std=%.3f lag=%.0fms', Rvals(k), sd, lag));
end
plot(S.t, S.qd_ref, 'w--', 'LineWidth',1, 'DisplayName','q̇ ref');
legend('TextColor','w','Color','k'); grid on;
xlabel('t (s)'); ylabel('q̇ (rad/s)');
title('R sweep: responsiveness vs smoothness','Color','w');


% load lab3_disturb.mat
% subplot(3,1,1); plot(t,q_out,'c');  ylabel('q (rad)');
% subplot(3,1,2); plot(t,qd_out,'g'); ylabel('q̇');
% subplot(3,1,3); plot(t,tau_d,'r');  ylabel('τ_d (N·m)'); xlabel('t (s)');