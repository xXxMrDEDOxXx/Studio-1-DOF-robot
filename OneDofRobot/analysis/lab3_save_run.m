function lab3_save_run(fname)
% LAB3_SAVE_RUN  เซฟ SDI run "ล่าสุด" เป็น .mat ไฟล์เดียว (ทุกสัญญาณ)
%
%   ใช้:  lab3_save_run('lab3_baseline.mat')
%
%   - ดึง run ล่าสุดจาก Simulation Data Inspector อัตโนมัติ
%   - เก็บแต่ละสัญญาณเป็นตัวแปรชื่อตาม signal name (เช่น qd_out, qd_fd, tau_d)
%   - เก็บเวลา common ไว้ในตัวแปร  t
%   - load('lab3_baseline.mat') แล้วได้ตัวแปรแยกครบเลย
%
%   *** ก่อนใช้: ตั้งชื่อ line ใน Simulink ให้ครบ (โดยเฉพาะ qd_fd/tau_d/i_est) ***

    ids = Simulink.sdi.getAllRunIDs;
    if isempty(ids)
        error('ไม่มี run ใน Data Inspector — กด Run เก็บข้อมูลก่อน');
    end
    run = Simulink.sdi.getRun(ids(end));     % run ล่าสุด

    L = struct;
    for k = 1:run.SignalCount
        s  = run.getSignalByIndex(k);
        nm = matlab.lang.makeValidName(s.Name);   % ชื่อให้เป็น field ได้
        L.(nm) = double(s.Values.Data(:));
    end
    L.t = double(run.getSignalByIndex(1).Values.Time(:));   % เวลา (วินาที)

    save(fname, '-struct', 'L');
    fprintf('เซฟ %d สัญญาณ -> %s\n', run.SignalCount, fname);
end
