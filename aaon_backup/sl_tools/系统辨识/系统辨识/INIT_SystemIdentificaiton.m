Ts_SystemIdentification.Ts_base = 0.012;
SYS_IDENTIFICATION_PARAM.wLength = 13; % 该参数无效
SYS_IDENTIFICATION_PARAM.nSettlingPeriod = 2; % 频点切换过渡周期数
SYS_IDENTIFICATION_PARAM.nEstimationPeriod = 4; % 频点扫频周期数
% 角速度扫频
SYS_IDENTIFICATION_PARAM.w_rad_rotorAngularVel = ...
    6*[0.05 0.1 0.5 1 2 3 4 6 8 10 20 30 40]; % 兴趣频点 [rad/s]
SYS_IDENTIFICATION_PARAM.amp_rotorAngularVel = 0.05; % 摄动幅值
% 垂速扫频
SYS_IDENTIFICATION_PARAM.w_rad_rotorVd = ...
    6*[0.05 0.1 0.5 1 2 3 4 6 8 10 20 30 40]; % 兴趣频点 [rad/s]
SYS_IDENTIFICATION_PARAM.amp_rotorVd = 0.05; % 摄动幅值