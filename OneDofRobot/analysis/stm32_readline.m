function y = stm32_readline()
%STM32_READLINE  อ่าน 1 บรรทัด CSV จาก serialport g_sp (base workspace) → 8x1 double
%  ใช้คู่กับ MATLAB Function block (เรียกแบบ coder.extrinsic)
%  g_sp เปิดไว้ใน model InitFcn:  g_sp = serialport("COM16",921600); ...
    y = zeros(8,1);
    raw = evalin('base', 'readline(g_sp)');      % string 1 บรรทัด (blocking → pace ~1kHz)
    v = sscanf(char(raw), '%d,%d,%d,%d,%d,%d,%d,%d');
    if numel(v) == 8
        y = double(v(:)) / 1000;                  % หาร scale (×1000 ฝั่ง STM32)
    end
end
