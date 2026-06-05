function stm32_to_simulink()
%% stm32_to_simulink — STM32 USART1 stream → Simulation Data Inspector (ดูค่าอย่างเดียว)
% ─────────────────────────────────────────────────────────────────────────
%  อ่าน stream (PB6=TX, 921600, CSV int×1000) เก็บไว้ → กด **Ctrl+C** หยุด
%  → ดันเข้า Data Inspector run "STM32 Live" (8 ช่อง) เปิดให้ดู/ซูม/วัด/export
%
%  เป็น function เพื่อให้ Ctrl+C เซฟได้ (onCleanup).  ต้องมี: Instrument Control + Simulink
% ─────────────────────────────────────────────────────────────────────────
clc;

%% ===================== CONFIG =====================
PORT  = "COM16";       % COM ของ TTL adapter
BAUD  = 921600;
Ts    = 0.001;         % firmware stream 1 kHz
MAXN  = 600000;        % buffer สูงสุด ~10 นาที @ 1kHz
NAMES = ["q_ref","qd_ref","qdd_ref","jerk","q_out","qd_out","qdd_out","V"];
% ==================================================

s = serialport(PORT, BAUD);
configureTerminator(s, "CR/LF");
s.Timeout = 2;
flush(s);

d = zeros(MAXN, 8);
k = 0;
finishup = onCleanup(@finalize);   %#ok<NASGU>  ดันเข้า DI ตอน Ctrl+C/จบ

fprintf("อัดอยู่... (กด Ctrl+C เพื่อหยุด + ส่งเข้า Data Inspector)\n");
tlast = tic;
while k < MAXN
    line = readline(s);
    if ~(isstring(line) || ischar(line)) || ismissing(line) || strlength(line)==0, continue; end
    v = sscanf(line, "%d,%d,%d,%d,%d,%d,%d,%d");
    if numel(v) ~= 8, continue; end
    k = k + 1;
    d(k,:) = v.' / 1000;
    if toc(tlast) >= 1, fprintf("  %d samples (%.1fs)\n", k, k*Ts); tlast = tic; end
end

%% ── nested: ตอนจบ → สร้าง DI run ──
    function finalize()
        clear s;                              % ปิด COM
        if k < 1, fprintf("\nไม่มีข้อมูล — เช็ค PORT/สาย\n"); return; end
        D = d(1:k,:);  t = (0:k-1).' * Ts;
        try
            tsc = arrayfun(@(i) timeseries(D(:,i), t), 1:8, 'UniformOutput', false);
            Simulink.sdi.createRun("STM32 Live", "namevalue", cellstr(NAMES), tsc);
            Simulink.sdi.view;
            fprintf("\n→ Data Inspector run ''STM32 Live'' (%d samples, %.2fs)\n", k, t(end));
        catch ME
            assignin('base','robot',timeseries(D,t));   % fallback
            fprintf("\n(SDI error: %s) → เก็บ ''robot'' ใน workspace แทน\n", ME.message);
        end
    end
end
