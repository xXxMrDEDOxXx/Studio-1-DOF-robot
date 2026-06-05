s = serialport("COM15", 921600); s.Timeout = 1;
write(s, uint8('A'), "uint8");
pause(0.05);
n = s.NumBytesAvailable        % ต้อง > 0
if n>0, disp(read(s, n, "uint8")); end   % ควรได้ 65 ('A')
clear s