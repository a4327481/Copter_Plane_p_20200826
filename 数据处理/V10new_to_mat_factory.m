%V10new_to_mat_factory
%V10 工厂日志分析脚本 
%20210607
%将指定数据 插值成同一时标
HD=180/pi;

%数据格式转换为mat格式并储存，仅转换一次
out_filePath=data_read_V10();
%数据格式转换为结构体，并补充为全名称
V10Log = V10_decode_auto(out_filePath);
%按工厂需求，将指定数据插值为同一长度
factory.time=V10Log.BUS_CTRL.Time_100us;

%% UBX
try
factory.UBLOX_numSV = interp1( V10Log.UBX_Raw.Time_100us , V10Log.UBX_Raw.numSV, factory.time,'previous');
factory.UBLOX_pDOP  = interp1( V10Log.UBX_Raw.Time_100us , V10Log.UBX_Raw.pDOP, factory.time,'previous');
%UBLOX 共存储下列数据，按需求插值。
% lat         
% lon         
% height      
% velN        
% velE        
% velD        
% hAcc        
% vAcc        
% sAcc        
% headAcc     
% pDOP        
% numSV       
% snr         
% svid               
catch ME
	disp(ME.message);
end

%% NRA
try
factory.Radar_Range = interp1( V10Log.Radar.Time_100us , V10Log.Radar.Range, factory.time,'previous');
factory.Radar_Flag  = interp1( V10Log.Radar.Time_100us , V10Log.Radar.Flag,  factory.time,'previous');
%NRA 共存储下列数据，按需求插值。
% lost_count
% Range     
% Flag      
% SNR       
% Rcs       
catch ME
	disp(ME.message);
end

%% MAG1
try
factory.Mag1_Left_true_data_x = interp1( V10Log.Mag1_Left.Time_100us,  V10Log.Mag1_Left.true_data_x, factory.time,'previous');
factory.Mag1_Left_true_data_y = interp1( V10Log.Mag1_Left.Time_100us,  V10Log.Mag1_Left.true_data_y, factory.time,'previous');
factory.Mag1_Left_true_data_z = interp1( V10Log.Mag1_Left.Time_100us,  V10Log.Mag1_Left.true_data_z, factory.time,'previous');
factory.Mag1_Left_err1        = interp1( V10Log.Mag1_Left.Time_100us,  V10Log.Mag1_Left.I2C_MagRetryCount_0(:,1), factory.time,'previous');
factory.Mag1_Left_err2        = interp1( V10Log.Mag1_Left.Time_100us,  V10Log.Mag1_Left.I2C_MagRetryCount_0(:,2), factory.time,'previous');
%MAG1 共存储下列数据，按需求插值。
% true_data_x                
% true_data_y                
% true_data_z                
% cali_data_x                
% cali_data_x                
% cali_data_y                
% cali_data_z                
% I2C_MagRetryCount[0][0]    
% I2C_MagRetryCount[0][1]    
catch ME
	disp(ME.message);
end

%% MAG2
try
factory.Mag2_Right_true_data_x = interp1( V10Log.Mag2_Right.Time_100us,  V10Log.Mag2_Right.true_data_x, factory.time,'previous');
factory.Mag2_Right_true_data_y = interp1( V10Log.Mag2_Right.Time_100us,  V10Log.Mag2_Right.true_data_y, factory.time,'previous');
factory.Mag2_Right_true_data_z = interp1( V10Log.Mag2_Right.Time_100us,  V10Log.Mag2_Right.true_data_z, factory.time,'previous');
factory.Mag2_Right_err1        = interp1( V10Log.Mag2_Right.Time_100us,  V10Log.Mag2_Right.I2C_MagRetryCount_1(:,1), factory.time,'previous');
factory.Mag2_Right_err2        = interp1( V10Log.Mag2_Right.Time_100us,  V10Log.Mag2_Right.I2C_MagRetryCount_1(:,2), factory.time,'previous');
%MAG2 共存储下列数据，按需求插值。
% true_data_x                
% true_data_y                
% true_data_z                
% cali_data_x                
% cali_data_x                
% cali_data_y                
% cali_data_z                
% I2C_MagRetryCount[0][0]    
% I2C_MagRetryCount[0][1]    
catch ME
	disp(ME.message);
end

%% LASE
try
factory.Laser_laser1_distance  = interp1( V10Log.Laser.Time_100us,  V10Log.Laser.laser1_distance, factory.time,'previous');
factory.Laser_laser2_distance  = interp1( V10Log.Laser.Time_100us,  V10Log.Laser.laser2_distance, factory.time,'previous');
%LASE 共存储下列数据，按需求插值。
% laser1_valid    
% laser1_distance 
% laser2_valid    
% laser2_distance 
% laser_flag      
catch ME
	disp(ME.message);
end

%% IMU1
try
factory.IMU1_Src_ax  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.ax, factory.time,'previous');
factory.IMU1_Src_ay  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.ay, factory.time,'previous');
factory.IMU1_Src_az  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.az, factory.time,'previous');
factory.IMU1_Src_gx  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.gx, factory.time,'previous');
factory.IMU1_Src_gy  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.gy, factory.time,'previous');
factory.IMU1_Src_gz  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.gz, factory.time,'previous');
factory.IMU1_Src_temperature  = interp1( V10Log.IMU1_Src.Time_100us,  V10Log.IMU1_Src.temperature, factory.time,'previous');
%IMU1 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz         
% temperature
% temp_pwm       
catch ME
	disp(ME.message);
end
%% IMU2
try
factory.IMU2_Src_ax  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.ax, factory.time,'previous');
factory.IMU2_Src_ay  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.ay, factory.time,'previous');
factory.IMU2_Src_az  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.az, factory.time,'previous');
factory.IMU2_Src_gx  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.gx, factory.time,'previous');
factory.IMU2_Src_gy  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.gy, factory.time,'previous');
factory.IMU2_Src_gz  = interp1( V10Log.IMU2_Src.Time_100us,  V10Log.IMU2_Src.gz, factory.time,'previous');
%IMU2 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz              
catch ME
	disp(ME.message);
end
%% IMF5
try
factory.IMU1_Logic_ax  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.ax, factory.time,'previous');
factory.IMU1_Logic_ay  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.ay, factory.time,'previous');
factory.IMU1_Logic_az  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.az, factory.time,'previous');
factory.IMU1_Logic_gx  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.gx, factory.time,'previous');
factory.IMU1_Logic_gy  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.gy, factory.time,'previous');
factory.IMU1_Logic_gz  = interp1( V10Log.IMU1_Logic.Time_100us,  V10Log.IMU1_Logic.gz, factory.time,'previous');
%IMF5 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz         
catch ME
	disp(ME.message);
end
%% IMF6
try
factory.IMU2_Logic_ax  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.ax, factory.time,'previous');
factory.IMU2_Logic_ay  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.ay, factory.time,'previous');
factory.IMU2_Logic_az  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.az, factory.time,'previous');
factory.IMU2_Logic_gx  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.gx, factory.time,'previous');
factory.IMU2_Logic_gy  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.gy, factory.time,'previous');
factory.IMU2_Logic_gz  = interp1( V10Log.IMU2_Logic.Time_100us,  V10Log.IMU2_Logic.gz, factory.time,'previous');
%IMF6 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz         
catch ME
	disp(ME.message);
end
%% IMF7
try
factory.IMU3_Logic_ax  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.ax, factory.time,'previous');
factory.IMU3_Logic_ay  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.ay, factory.time,'previous');
factory.IMU3_Logic_az  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.az, factory.time,'previous');
factory.IMU3_Logic_gx  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.gx, factory.time,'previous');
factory.IMU3_Logic_gy  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.gy, factory.time,'previous');
factory.IMU3_Logic_gz  = interp1( V10Log.IMU3_Logic.Time_100us,  V10Log.IMU3_Logic.gz, factory.time,'previous');
%IMF7 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz         
catch ME
	disp(ME.message);
end
%% IMF8
try
factory.IMU4_Logic_ax  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.ax, factory.time,'previous');
factory.IMU4_Logic_ay  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.ay, factory.time,'previous');
factory.IMU4_Logic_az  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.az, factory.time,'previous');
factory.IMU4_Logic_gx  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.gx, factory.time,'previous');
factory.IMU4_Logic_gy  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.gy, factory.time,'previous');
factory.IMU4_Logic_gz  = interp1( V10Log.IMU4_Logic.Time_100us,  V10Log.IMU4_Logic.gz, factory.time,'previous');
%IMF8 共存储下列数据，按需求插值。
% ax         
% ay         
% az         
% gx         
% gy         
% gz         
catch ME
	disp(ME.message);
end

%% ECS1
try
factory.ECS1.ecs_rpm1  = interp1( V10Log.ECS1.Time_100us,  V10Log.ECS1.ecs_rpm(:,1),  factory.time,'previous');
factory.ECS1.ecs_rpm2  = interp1( V10Log.ECS1.Time_100us,  V10Log.ECS1.ecs_rpm(:,2),  factory.time,'previous');
factory.ECS1.ecs_rpm3  = interp1( V10Log.ECS1.Time_100us,  V10Log.ECS1.ecs_rpm(:,3),  factory.time,'previous');
%ECS1 共存储下列数据，按需求插值。
% state[0]   
% state[1]   
% state[2]   
% ecs_rpm[0] 
% ecs_rpm[1] 
% ecs_rpm[2] 
% ecs_imv[0] 
% ecs_imv[1] 
% ecs_imv[2] 
% ecs_ppm[0] 
% ecs_ppm[1] 
% ecs_ppm[2]           
catch ME
	disp(ME.message);
end

%% ECS2
try
factory.ECS2.ecs_rpm4  = interp1( V10Log.ECS2.Time_100us,  V10Log.ECS2.ecs_rpm(:,1),  factory.time,'previous');
factory.ECS2.ecs_rpm5  = interp1( V10Log.ECS2.Time_100us,  V10Log.ECS2.ecs_rpm(:,2),  factory.time,'previous');
%ECS2 共存储下列数据，按需求插值。
% state[0]     
% state[1]     
% ecs_rpm[0]   
% ecs_rpm[1]   
% ecs_imv[0]   
% ecs_imv[1]   
% ecs_ppm[0]   
% ecs_ppm[1]   
% flag[0]      
% flag[1]      
% flag[2]      
% flag[3]      
% flag[4]      
          
catch ME
	disp(ME.message);
end

%% BAR1
try
factory.Barometer1_pressure  = interp1( V10Log.Barometer1.Time_100us,     V10Log.Barometer1.pressure,     factory.time,'previous');
factory.Barometer1_temperature  = interp1( V10Log.Barometer1.Time_100us,  V10Log.Barometer1.temperature,  factory.time,'previous');

%BAR1 共存储下列数据，按需求插值。
% ground_pressure    
% ground_temperature 
% pressure           
% altitude           
% temperature                 
catch ME
	disp(ME.message);
end

%% BAR2
try
factory.Barometer2_pressure  = interp1( V10Log.Barometer2.Time_100us,     V10Log.Barometer2.pressure,     factory.time,'previous');
factory.Barometer2_temperature  = interp1( V10Log.Barometer2.Time_100us,  V10Log.Barometer2.temperature,  factory.time,'previous');

%BAR2 共存储下列数据，按需求插值。
% ground_pressure    
% ground_temperature 
% pressure           
% altitude           
% temperature                 
catch ME
	disp(ME.message);
end

%% ARP1
try
factory.AirSpeed1Head_indicated_airspeed  = interp1( V10Log.AirSpeed1Head.Time_100us,     V10Log.AirSpeed1Head.indicated_airspeed,     factory.time,'previous');
factory.AirSpeed1Head_true_airspeed       = interp1( V10Log.AirSpeed1Head.Time_100us,     V10Log.AirSpeed1Head.true_airspeed,          factory.time,'previous');
factory.V10Log.AirSpeed1Head.err1         = interp1( V10Log.AirSpeed1Head.Time_100us,     V10Log.AirSpeed1Head.I2C_AirRetryCount_0(:,1),          factory.time,'previous');
factory.V10Log.AirSpeed1Head.err2         = interp1( V10Log.AirSpeed1Head.Time_100us,     V10Log.AirSpeed1Head.I2C_AirRetryCount_0(:,2),          factory.time,'previous');

%ARP1 共存储下列数据，按需求插值。
% air_temp               
% air_diff_press_pa_raw  
% indicated_airspeed     
% true_airspeed          
% EAS_Algo               
% EAS2TAS_Algo           
% I2C_AirRetryCount[0][0]
% I2C_AirRetryCount[0][1]             
catch ME
	disp(ME.message);
end

%% ARP2
try
factory.AirSpeed2Wing_indicated_airspeed  = interp1( V10Log.AirSpeed2Wing.Time_100us,     V10Log.AirSpeed2Wing.indicated_airspeed,     factory.time,'previous');
factory.AirSpeed2Wing_true_airspeed       = interp1( V10Log.AirSpeed2Wing.Time_100us,     V10Log.AirSpeed2Wing.true_airspeed,          factory.time,'previous');
factory.V10Log.AirSpeed2Wing.err1         = interp1( V10Log.AirSpeed2Wing.Time_100us,     V10Log.AirSpeed2Wing.I2C_AirRetryCount_1(:,1),          factory.time,'previous');
factory.V10Log.AirSpeed2Wing.err2         = interp1( V10Log.AirSpeed2Wing.Time_100us,     V10Log.AirSpeed2Wing.I2C_AirRetryCount_1(:,2),          factory.time,'previous');

%ARP2 共存储下列数据，按需求插值。
% air_temp               
% air_diff_press_pa_raw  
% indicated_airspeed     
% true_airspeed          
% EAS_Algo               
% EAS2TAS_Algo           
% I2C_AirRetryCount[0][0]
% I2C_AirRetryCount[0][1]             
catch ME
	disp(ME.message);
end

%% AL26
try
factory.OUT_NAVI2CONTROL_yawd      = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.yawd,       factory.time,'previous');
factory.OUT_NAVI2CONTROL_pitchd    = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.pitchd,     factory.time,'previous');
factory.OUT_NAVI2CONTROL_rolld     = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.rolld,      factory.time,'previous');
factory.OUT_NAVI2CONTROL_latd      = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.yawd,       factory.time,'previous');
factory.OUT_NAVI2CONTROL_lond      = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.latd,     factory.time,'previous');
factory.OUT_NAVI2CONTROL_rolld     = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.lond,      factory.time,'previous');
factory.OUT_NAVI2CONTROL_alt       = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.alt,       factory.time,'previous');
factory.OUT_NAVI2CONTROL_Vn        = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.Vn,     factory.time,'previous');
factory.OUT_NAVI2CONTROL_Vn        = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.Ve,     factory.time,'previous');
factory.OUT_NAVI2CONTROL_Vn        = interp1( V10Log.OUT_NAVI2CONTROL.Time_100us,     V10Log.OUT_NAVI2CONTROL.Vd,     factory.time,'previous');

%ARP2 共存储下列数据，按需求插值。
% yawd       
% pitchd     
% rolld      
% latd       
% lond       
% alt        
% Vn         
% Ve         
% Vd                   
catch ME
	disp(ME.message);
end

%% BT00 BT01 BT02
try
factory.Battery01_Voltage1    = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Voltage(:,1),    factory.time,'previous');
factory.Battery01_Voltage2    = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Voltage(:,2),    factory.time,'previous');
factory.Battery01_Cell_Volt1  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery01_Cell_Volt2  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery01_Cell_Volt3  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery01_Cell_Volt4  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery01_Cell_Volt5  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery01_Cell_Volt6  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery01_Cell_Volt7  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery01_Cell_Volt8  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery01_Average_Current  = interp1( V10Log.Battery01.Time_100us,  V10Log.Battery01.Average_Current,  factory.time,'previous');
%BT00 BT01 BT02 共存储下列数据，按需求插值。
% bat_pack_num          
% Design_Cap            
% Design_Volt           
% Manufacture_Date      
% Serial_Num            
% FW_Version            
% Run_Time_To_Empty     
% Average_Time_To_Empty 
% Rx_Count              
% Cycle_Count           
% Total_Cycles          
% SOH                   
% Temperature[0] 
% Temperature[1] 
% Temperature[2] 
% ASOC           
% RSOC           
% Rem_Cap        
% FCC            
% Safty_Status   
% Other_Status   
% Balance_Status 
% Power_State    
% flag           
% Voltage[0]     
% Voltage[1]     
% Cell_Volt[0]   
% Cell_Volt[1]   
% Cell_Volt[2]   
% Cell_Volt[3]   
% Cell_Volt[4]   
% Cell_Volt[5]   
% Cell_Volt[6]   
% Cell_Volt[7]   
% Now_Current[0] 
% Now_Current[1] 
% Average_Current       
catch ME
	disp(ME.message);
end
%% BT10 BT11 BT12
try
factory.Battery02_Voltage1    = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Voltage(:,1),    factory.time,'previous');
factory.Battery02_Voltage2    = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Voltage(:,2),    factory.time,'previous');
factory.Battery02_Cell_Volt1  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery02_Cell_Volt2  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery02_Cell_Volt3  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery02_Cell_Volt4  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery02_Cell_Volt5  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery02_Cell_Volt6  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery02_Cell_Volt7  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery02_Cell_Volt8  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery02_Average_Current  = interp1( V10Log.Battery02.Time_100us,  V10Log.Battery02.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end
%% BT20 BT21 BT22
try
factory.Battery03_Voltage1    = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Voltage(:,1),    factory.time,'previous');
factory.Battery03_Voltage2    = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Voltage(:,2),    factory.time,'previous');
factory.Battery03_Cell_Volt1  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery03_Cell_Volt2  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery03_Cell_Volt3  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery03_Cell_Volt4  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery03_Cell_Volt5  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery03_Cell_Volt6  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery03_Cell_Volt7  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery03_Cell_Volt8  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery03_Average_Current  = interp1( V10Log.Battery03.Time_100us,  V10Log.Battery03.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end
%% BT30 BT31 BT32
try
factory.Battery04_Voltage1    = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Voltage(:,1),    factory.time,'previous');
factory.Battery04_Voltage2    = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Voltage(:,2),    factory.time,'previous');
factory.Battery04_Cell_Volt1  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery04_Cell_Volt2  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery04_Cell_Volt3  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery04_Cell_Volt4  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery04_Cell_Volt5  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery04_Cell_Volt6  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery04_Cell_Volt7  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery04_Cell_Volt8  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery04_Average_Current  = interp1( V10Log.Battery04.Time_100us,  V10Log.Battery04.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end

%% BT40 BT41 BT42
try
factory.Battery05_Voltage1    = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Voltage(:,1),    factory.time,'previous');
factory.Battery05_Voltage2    = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Voltage(:,2),    factory.time,'previous');
factory.Battery05_Cell_Volt1  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery05_Cell_Volt2  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery05_Cell_Volt3  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery05_Cell_Volt4  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery05_Cell_Volt5  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery05_Cell_Volt6  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery05_Cell_Volt7  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery05_Cell_Volt8  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery05_Average_Current  = interp1( V10Log.Battery05.Time_100us,  V10Log.Battery05.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end

%% BT50 BT51 BT52
try
factory.Battery06_Voltage1    = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Voltage(:,1),    factory.time,'previous');
factory.Battery06_Voltage2    = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Voltage(:,2),    factory.time,'previous');
factory.Battery06_Cell_Volt1  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery06_Cell_Volt2  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery06_Cell_Volt3  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery06_Cell_Volt4  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery06_Cell_Volt5  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery06_Cell_Volt6  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery06_Cell_Volt7  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery06_Cell_Volt8  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery06_Average_Current  = interp1( V10Log.Battery06.Time_100us,  V10Log.Battery06.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end

%% BT60 BT61 BT62
try
factory.Battery07_Voltage1    = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Voltage(:,1),    factory.time,'previous');
factory.Battery07_Voltage2    = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Voltage(:,2),    factory.time,'previous');
factory.Battery07_Cell_Volt1  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery07_Cell_Volt2  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery07_Cell_Volt3  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery07_Cell_Volt4  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery07_Cell_Volt5  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery07_Cell_Volt6  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery07_Cell_Volt7  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery07_Cell_Volt8  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery07_Average_Current  = interp1( V10Log.Battery07.Time_100us,  V10Log.Battery07.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end


%% BT70 BT71 BT72
try
factory.Battery08_Voltage1    = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Voltage(:,1),    factory.time,'previous');
factory.Battery08_Voltage2    = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Voltage(:,2),    factory.time,'previous');
factory.Battery08_Cell_Volt1  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery08_Cell_Volt2  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery08_Cell_Volt3  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery08_Cell_Volt4  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery08_Cell_Volt5  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery08_Cell_Volt6  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery08_Cell_Volt7  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery08_Cell_Volt8  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery08_Average_Current  = interp1( V10Log.Battery08.Time_100us,  V10Log.Battery08.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end

%% BT80 BT81 BT82
try
factory.Battery09_Voltage1    = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Voltage(:,1),    factory.time,'previous');
factory.Battery09_Voltage2    = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Voltage(:,2),    factory.time,'previous');
factory.Battery09_Cell_Volt1  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery09_Cell_Volt2  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery09_Cell_Volt3  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery09_Cell_Volt4  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery09_Cell_Volt5  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery09_Cell_Volt6  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery09_Cell_Volt7  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery09_Cell_Volt8  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery09_Average_Current  = interp1( V10Log.Battery09.Time_100us,  V10Log.Battery09.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end

%% BT90 BT91 BT92
try
factory.Battery10_Voltage1    = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Voltage(:,1),    factory.time,'previous');
factory.Battery10_Voltage2    = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Voltage(:,2),    factory.time,'previous');
factory.Battery10_Cell_Volt1  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,1),  factory.time,'previous');
factory.Battery10_Cell_Volt2  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,2),  factory.time,'previous');
factory.Battery10_Cell_Volt3  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,3),  factory.time,'previous');
factory.Battery10_Cell_Volt4  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,4),  factory.time,'previous');
factory.Battery10_Cell_Volt5  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,5),  factory.time,'previous');
factory.Battery10_Cell_Volt6  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,6),  factory.time,'previous');
factory.Battery10_Cell_Volt7  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,7),  factory.time,'previous');
factory.Battery10_Cell_Volt8  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Cell_Volt(:,8),  factory.time,'previous');
factory.Battery10_Average_Current  = interp1( V10Log.Battery10.Time_100us,  V10Log.Battery10.Average_Current,  factory.time,'previous');   
catch ME
	disp(ME.message);
end