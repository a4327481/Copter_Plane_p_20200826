%V1000 to txt  
% Display surf plot of the currently selected data.
%mat 格式转换为画图
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
else
    [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
end
if FileName==0
    return;
end
BLOCK_SIZE=288;

fp=fopen([PathName,'\\',FileName],'r');
        data = fread(fp);
        fclose(fp);
        n=length(data);
        i=1;
        m=floor(n/BLOCK_SIZE);
        data=reshape(data(1:BLOCK_SIZE*m)',[BLOCK_SIZE,m]);
        data=data';
    
%         Count=binDecode(data,1,0,0);    
%         indexn=find(Count==1);
%         data=data(indexn(1):end,:);
        
%         Count=binDecode(data,1,0,0);     
%         indexn=find(Count>0);
%         if( mod(Count(indexn(1)),2) ==1 )
%             data=data(indexn(1):end,:);
%         else
%             data=data( (indexn(1)+1) :end,:);
%         end
        Count=binDecode(data,1,0,0);     
        indexn=find(mod(Count,8)==1);
        if( indexn(1) >1 )
            data=data(indexn(1):end,:);
        end
               
        [m,mm]=size(data);
        m=floor(m/16)*16;
        data=data(1:m,:);
        datacolumn=data;
        Count=binDecode(data,1,0,0); 
        HD=180/pi;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SYSTEM
        
        temp=reshape([data(:,251:254)'],1,[]);
        save_time=double(typecast(uint8(temp),'uint32')')*1e-3;
        temp=reshape([data(8:8:end,255:256)'],1,[]);        
        FC_VERSION0=(typecast(uint8(temp),'uint16'))';
        FC_VERSION  =[FC_VERSION0; FC_VERSION0];
        FC_VERSION  (1:2:end)=FC_VERSION0;
        FC_VERSION  (2:2:end)=FC_VERSION0;    
        temp=reshape([data(4:16:end,243:246)'],1,[]);        
        fc_sn_num=(typecast(uint8(temp),'uint32'))';  
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %IMU
        temp=reshape([data(:,3:6)'],1,[]);
        ax=typecast(uint8(temp),'single')';
        temp=reshape([data(:,7:10)'],1,[]);
        ay=-typecast(uint8(temp),'single')';
        temp=reshape([data(:,11:14)'],1,[]);
        az=-typecast(uint8(temp),'single')';      
        temp=reshape([data(:,15:18)'],1,[]);
        gx=typecast(uint8(temp),'single')'*HD;     
        temp=reshape([data(:,19:22)'],1,[]);
        gy=-typecast(uint8(temp),'single')'*HD;   
        temp=reshape([data(:,23:26)'],1,[]);
        gz=-typecast(uint8(temp),'single')'*HD;
        

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %baro
        temp=reshape([data(1:2:end,41:42)'],1,[]);
        altitue=double(typecast(uint8(temp),'int16')')/32768*2000;        
        temp=reshape([data(1:2:end,43:44)'],1,[]);
        temperature=double(typecast(uint8(temp),'int16')')/32768*100;        
        temp=reshape([data(1:2:end,45:46)'],1,[]);
        pressure=double(typecast(uint8(temp),'int16')')/32768*1100;             
        temp=reshape([data(1:2:end,47:48)'],1,[]);
        temperature_gs=double(typecast(uint8(temp),'int16')')/32768*1100;           
        temp=reshape([data(1:2:end,49:50)'],1,[]);
        pressure_gs=double(typecast(uint8(temp),'int16')')/32768*1100;  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Mag

        % mag1
        temp=reshape([data(4:4:end,109:110)'],1,[]);
        mag1_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,111:112)'],1,[]);
        mag1_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,113:114)'],1,[]);
        mag1_z=double(typecast(uint8(temp),'int16')')/32768*2;  
        temp=mag1_x;
        mag1_x=-mag1_y;
        mag1_y=-temp;
        mag1_z=-mag1_z;
        % mag2
        temp=reshape([data(4:4:end,115:116)'],1,[]);
        mag2_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,117:118)'],1,[]);
        mag2_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,119:120)'],1,[]);
        mag2_z=double(typecast(uint8(temp),'int16')')/32768*2;    
        
        temp=mag2_x;
        mag2_x=-mag2_y;
        mag2_y=-temp;
        mag2_z=-mag2_z;
        % mag3
        temp=reshape([data(4:4:end,121:122)'],1,[]);
        mag3_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,123:124)'],1,[]);
        mag3_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,125:126)'],1,[]);
        mag3_z=double(typecast(uint8(temp),'int16')')/32768*2;
%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         temp=reshape([data(1:2:end,191:192)'],1,[]);
%         mag2_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
%         temp=reshape([data(1:2:end,193:194)'],1,[]);
%         mag2_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
%         temp=reshape([data(1:2:end,195:196)'],1,[]);
%         mag2_zx=double(typecast(uint8(temp),'int16')')/32768*2;
%         mag2_xx=mag2_xx(1:2:end);
%         mag2_yx=mag2_yx(1:2:end);
%         mag2_zx=mag2_zx(1:2:end);
%         temp=mag2_xx;
%         mag2_xx=-mag2_yx;
%         mag2_yx=-temp;
%         mag2_zx=-mag2_zx;
%         
%         temp=reshape([data(4:4:end,131:132)'],1,[]);
%         mag1_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
%         temp=reshape([data(4:4:end,133:134)'],1,[]);
%         mag1_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
%         temp=reshape([data(4:4:end,135:136)'],1,[]);
%         mag1_zx=double(typecast(uint8(temp),'int16')')/32768*2; 
%         temp=mag1_xx;
%         mag1_xx=-mag1_yx;
%         mag1_yx=-temp;
%         mag1_zx=-mag1_zx;
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        temp=reshape([data(1:2:end,191:192)'],1,[]);
        mag2_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(1:2:end,193:194)'],1,[]);
        mag2_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(1:2:end,195:196)'],1,[]);
        mag2_zx=double(typecast(uint8(temp),'int16')')/32768*2;
        mag2_xx=mag2_xx(1:2:end);
        mag2_yx=mag2_yx(1:2:end);
        mag2_zx=mag2_zx(1:2:end);
        temp=mag2_xx;
        mag2_xx=-mag2_yx;
        mag2_yx=-temp;
        mag2_zx=-mag2_zx;
        
        temp=reshape([data(1:2:end,167:168)'],1,[]);
        mag1_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(1:2:end,169:170)'],1,[]);
        mag1_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(1:2:end,171:172)'],1,[]);
        mag1_zx=double(typecast(uint8(temp),'int16')')/32768*2;
        mag1_xx=mag1_xx(1:2:end);
        mag1_yx=mag1_yx(1:2:end);
        mag1_zx=mag1_zx(1:2:end);
        temp=mag1_xx;
        mag1_xx=-mag1_yx;
        mag1_yx=-temp;
        mag1_zx=-mag1_zx;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %radar
        temp=reshape([data(2:4:end,109)'],1,[]);
        radar_SNR=uint8(temp');
        temp=reshape([data(2:4:end,110)'],1,[]);
        radar_Flag=uint8(temp');
        temp=reshape([data(2:4:end,111:112)'],1,[]);
        radar_Range=double(typecast(uint8(temp),'int16')')/32768*2000;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %ublox
        temp=reshape([data(1:4:end,109:112)'],1,[]);
        ublox_iTOW=double(typecast(uint8(temp),'int32')');
        temp=reshape([data(1:4:end,113:116)'],1,[]);
        ublox_velE=double(typecast(uint8(temp),'int32')')*1e-3;
        temp=reshape([data(1:4:end,117:120)'],1,[]);
        ublox_velN=double(typecast(uint8(temp),'int32')')*1e-3;       
        temp=reshape([data(1:4:end,121:124)'],1,[]);
        ublox_velD=double(typecast(uint8(temp),'int32')')*1e-3;         
        temp=reshape([data(1:4:end,125:128)'],1,[]);
        ublox_lon=double(typecast(uint8(temp),'int32')')*1e-7;  
        temp=reshape([data(1:4:end,129:132)'],1,[]);
        ublox_lat=double(typecast(uint8(temp),'int32')')*1e-7;    
        temp=reshape([data(1:4:end,133:136)'],1,[]);
        ublox_height=double(typecast(uint8(temp),'int32')')*1e-3;   
        temp=reshape([data(4:4:end,127:128)'],1,[]);
        ublox_pDOP=double(typecast(uint8(temp),'uint16')')*1e-2; 
        temp=reshape([data(4:4:end,129)'],1,[]);
        ublox_numSV=typecast(uint8(temp),'uint8')';
        ublox_V=hypot(ublox_velN,ublox_velE);
        ublox_theta=-atan2d(ublox_velD,ublox_V);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %algo
        temp=reshape([data(2:2:end,41)'],1,[]);
        Switch_LockStatus=typecast(uint8(temp),'uint8')';         
        temp=reshape([data(2:2:end,42)'],1,[]);
        Lock_Status=typecast(uint8(temp),'uint8')';    
        temp=reshape([data(2:2:end,43)'],1,[]);
        algo_mode=typecast(uint8(temp),'uint8')';  
        temp=reshape([data(2:2:end,44)'],1,[]);
        PLANE_COPTER_MODE=typecast(uint8(temp),'uint8')';          
        temp=reshape([data(2:2:end,47:48)'],1,[]);
        src_pitch=double(typecast(uint8(temp),'uint16')');   
        temp=reshape([data(2:2:end,49:50)'],1,[]);
        src_roll=double(typecast(uint8(temp),'uint16')'); 
        temp=reshape([data(2:2:end,51:52)'],1,[]);
        src_throttle=double(typecast(uint8(temp),'uint16')');         
        temp=reshape([data(2:2:end,53:54)'],1,[]);
        src_yaw=double(typecast(uint8(temp),'uint16')');     
        temp=reshape([data(2:2:end,55:56)'],1,[]);
        algo_remote_ct_st_pitch=double(typecast(uint8(temp),'int16')')/32768*80; 
        temp=reshape([data(2:2:end,57:58)'],1,[]);
        algo_remote_ct_st_roll=double(typecast(uint8(temp),'int16')')/32768*80;           
        temp=reshape([data(2:2:end,59:60)'],1,[]);
        algo_remote_ct_st_throttle=double(typecast(uint8(temp),'int16')')/32768*400;           
        temp=reshape([data(2:2:end,61:62)'],1,[]);
        algo_remote_ct_st_yaw=double(typecast(uint8(temp),'int16')')/32768*200;           
        temp=reshape([data(2:2:end,63:64)'],1,[]);
        algo_roll=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(2:2:end,65:66)'],1,[]);
        algo_pitch=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(2:2:end,67:68)'],1,[]);
        algo_yaw=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(2:2:end,69:70)'],1,[]);
        algo_curr_vel_0=double(typecast(uint8(temp),'int16')')/32768*30;           
        temp=reshape([data(2:2:end,71:72)'],1,[]);
        algo_curr_vel_1=double(typecast(uint8(temp),'int16')')/32768*30;
        temp=reshape([data(2:2:end,73:74)'],1,[]);
        algo_curr_vel_2=double(typecast(uint8(temp),'int16')')/32768*30; 
        temp=reshape([data(2:2:end,75:76)'],1,[]);
        algo_curr_pos_0=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,77:78)'],1,[]);
        algo_curr_pos_1=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,79:80)'],1,[]);
        algo_curr_pos_2=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,81:82)'],1,[]);
        algo_curr_alt=double(typecast(uint8(temp),'int16')')/32768*10000*10/100;%%%%%%%%%%%%%%%%20200215
        temp=reshape([data(2:2:end,83:84)'],1,[]);
        algo_rate_target_ang_vel_0=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(2:2:end,85:86)'],1,[]);
        algo_rate_target_ang_vel_1=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(2:2:end,87:88)'],1,[]);
        algo_rate_target_ang_vel_2=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(2:2:end,89:90)'],1,[]);
        algo_roll_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(2:2:end,91:92)'],1,[]);
        algo_pitch_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(2:2:end,93:94)'],1,[]);
        algo_yaw_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(2:2:end,95:96)'],1,[]);
        algo_throttle_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(2:2:end,97:98)'],1,[]);
        algo_attitude_target_euler_rate_0=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(2:2:end,99:100)'],1,[]);
        algo_attitude_target_euler_rate_1=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(2:2:end,101:102)'],1,[]);
        algo_attitude_target_euler_rate_2=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(2:2:end,103:104)'],1,[]);
        algo_attitude_target_euler_angle_0=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(2:2:end,105:106)'],1,[]);
        algo_attitude_target_euler_angle_1=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(2:2:end,107:108)'],1,[]);
        algo_attitude_target_euler_angle_2=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(1:2:end,51:52)'],1,[]);
        algo_pwm_out_1=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(1:2:end,53:54)'],1,[]);
        algo_pwm_out_2=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(1:2:end,55:56)'],1,[]);
        algo_pwm_out_3=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(1:2:end,57:58)'],1,[]);
        algo_pwm_out_4=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(1:2:end,59:60)'],1,[]);
        algo_attitude_error_vector_0=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(1:2:end,61:62)'],1,[]);
        algo_attitude_error_vector_1=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(1:2:end,63:64)'],1,[]);
        algo_attitude_error_vector_2=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(1:2:end,65:66)'],1,[]);
        algo_speed_out_0=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(1:2:end,67:68)'],1,[]);
        algo_speed_out_1=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(1:2:end,69:70)'],1,[]);
        algo_speed_out_2=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(1:2:end,71:72)'],1,[]);
        algo_speed_out_3=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(1:2:end,73:74)'],1,[]);
        algo_pos_target_0=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:2:end,75:76)'],1,[]);
        algo_pos_target_1=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:2:end,77:78)'],1,[]);
        algo_pos_target_2=double(typecast(uint8(temp),'int16')')/32768*100*10;%%20200215
        temp=reshape([data(1:2:end,79:80)'],1,[]);
        algo_vel_target_0=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(1:2:end,81:82)'],1,[]);
        algo_vel_target_1=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(1:2:end,83:84)'],1,[]);
        algo_vel_target_2=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(1:2:end,85:86)'],1,[]);
        algo_accel_target_0=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(1:2:end,87:88)'],1,[]);
        algo_accel_target_1=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(1:2:end,89:90)'],1,[]);
        algo_accel_target_2=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(1:2:end,91:92)'],1,[]);
        algo_z_accel_meas=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(1:2:end,93:94)'],1,[]);
        algo_climb_rate_cms=double(typecast(uint8(temp),'int16')')/32768*5;
        temp=reshape([data(1:2:end,95:96)'],1,[]);
        algo_throttle_filter=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(1:2:end,97:98)'],1,[]);
        algo_vel_desired_2=double(typecast(uint8(temp),'int16')')/32768*10; 
        temp=reshape([data(1:2:end,99)'],1,[]);
        temp1=typecast(uint8(temp),'uint8');
        algo_throttle_lower=bitand(temp1,1)/1;
        algo_throttle_upper=bitand(temp1,2^1)/2^1;
        algo_limit_pos_up=bitand(temp1,2^2)/2^2;
        temp=reshape([data(1:2:end,101:102)'],1,[]);
        algo_nav_pitch_cd=double(typecast(uint8(temp),'int16')')/32768*3600;        
        temp=reshape([data(1:2:end,103:104)'],1,[]);
        algo_latAccDem=double(typecast(uint8(temp),'int16')')/32768*20;
        temp=reshape([data(1:2:end,105:106)'],1,[]);
        algo_yaw_out=double(typecast(uint8(temp),'int16')')/32768*4500; 
        temp=reshape([data(1:2:end,107:108)'],1,[]);
        algo_throttle_dem=double(typecast(uint8(temp),'int16')')/32768*1;         
        temp=reshape([data(2:2:end,137:138)'],1,[]);
        algo_k_rudder=double(typecast(uint8(temp),'int16')')/32768*45;         
        temp=reshape([data(2:2:end,139:140)'],1,[]);
        algo_k_elevator=double(typecast(uint8(temp),'int16')')/32768*45;
        temp=reshape([data(2:2:end,141:142)'],1,[]);
        algo_k_throttle=double(typecast(uint8(temp),'int16')')/32768*1; 
        temp=reshape([data(2:2:end,143:144)'],1,[]);
        algo_k_aileron=double(typecast(uint8(temp),'int16')')/32768*45; 
        temp=reshape([data(1:2:end,137:138)'],1,[]);
        algo_servo_out_0=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:2:end,139:140)'],1,[]);
        algo_servo_out_1=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:2:end,141:142)'],1,[]);
        algo_servo_out_2=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:2:end,143:144)'],1,[]);
        algo_servo_out_3=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:2:end,177:178)'],1,[]);
        algo_pitch_dem=double(typecast(uint8(temp),'int16')')/32768*3.15*HD; 
        temp=reshape([data(1:2:end,179:180)'],1,[]);
        algo_hgt_dem=double(typecast(uint8(temp),'int16')')/32768*1000; 
        temp=reshape([data(1:2:end,181:182)'],1,[]);
        algo_throttle_dem=double(typecast(uint8(temp),'int16')')/32768*1.1; 
        temp=reshape([data(1:2:end,183:184)'],1,[]);
        algo_arspeed=double(typecast(uint8(temp),'int16')')/32768*50; 
        temp=reshape([data(2:2:end,195:196)'],1,[]);
        algo_roll_target_pilot =double(typecast(uint8(temp),'int16')')/32768*50;        
        temp=reshape([data(1:2:end,185:186)'],1,[]);
        algo_pitch_target_pilot=double(typecast(uint8(temp),'int16')')/32768*50;
        temp=reshape([data(2:2:end,165:168)'],1,[]);
        algo_current_loc_0 =double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(2:2:end,169:172)'],1,[]);
        algo_current_loc_1 =double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(2:2:end,79:80)'],1,[]);
        algo_tail_tilt =double(typecast(uint8(temp),'int16')')/100; 
        temp=reshape([data(1:2:end,165:166)'],1,[]);
        algo_tail_tilt_pwm =double(typecast(uint8(temp),'int16')');     
        
        algo_nav_roll=cosd(algo_pitch).*atan(algo_latAccDem * 0.101972) *HD;
        algo_nav_pitch=algo_nav_pitch_cd/100;
        temp=reshape([data(1:2:end,151:152)'],1,[]);
        algo_roll_target=double(typecast(uint8(temp),'int16')')/32768*50; 
        temp=reshape([data(1:2:end,153:154)'],1,[]);
        algo_pitch_target=double(typecast(uint8(temp),'int16')')/32768*50;  
        temp=reshape([data(2:2:end,175:176)'],1,[]);
        algo_weathervane_last_output=double(typecast(uint8(temp),'int16')')/32768*1;         
        temp=reshape([data(4:4:end,220)'],1,[]);
        algo_PathMode0=double(typecast(uint8(temp),'uint8')'); 
        algo_PathMode=[algo_PathMode0; algo_PathMode0];
        algo_PathMode(1:2:end)=algo_PathMode0;
        algo_PathMode(2:2:end)=algo_PathMode0;
        temp=reshape([data(1:4:end,221:222)'],1,[]);
        algo_heightCmd0 =double(typecast(uint8(temp),'int16')')/32768*300; 
        algo_heightCmd =[algo_heightCmd0; algo_heightCmd0];
        algo_heightCmd (1:2:end)=algo_heightCmd0;
        algo_heightCmd (2:2:end)=algo_heightCmd0;
        temp=reshape([data(2:4:end,221:222)'],1,[]);
        algo_maxClimbSpeed0 =double(typecast(uint8(temp),'int16')')/32768*500; 
        algo_maxClimbSpeed =[algo_maxClimbSpeed0; algo_maxClimbSpeed0];
        algo_maxClimbSpeed (1:2:end)=algo_maxClimbSpeed0;
        algo_maxClimbSpeed (2:2:end)=algo_maxClimbSpeed0;
        temp=reshape([data(3:4:end,209)'],1,[]);
        algo_flightTaskMode0 =double(typecast(uint8(temp),'uint8')'); 
        algo_flightTaskMode =[algo_flightTaskMode0; algo_flightTaskMode0];
        algo_flightTaskMode (1:2:end)=algo_flightTaskMode0;
        algo_flightTaskMode (2:2:end)=algo_flightTaskMode0;    
        
        temp=reshape([data(8:8:end,205)'],1,[]);
        GSConnected0 =double(typecast(uint8(temp),'uint8')'); 
        GSConnected =[GSConnected0; GSConnected0; GSConnected0; GSConnected0];
        GSConnected (1:4:end)=GSConnected0;
        GSConnected (2:4:end)=GSConnected0; 
        GSConnected (3:4:end)=GSConnected0; 
        GSConnected (4:4:end)=GSConnected0; 

        temp=reshape([data(1:2:end,187)'],1,[]);
        Remote_Disconnect_Status =double(typecast(uint8(temp),'uint8')'); 
 
        
        temp=reshape([data(4:4:end,135:136)'],1,[]);
        remote_time_cnt0  =double(typecast(uint8(temp),'int16')'); 
        remote_time_cnt  =[remote_time_cnt0; remote_time_cnt0];
        remote_time_cnt  (1:2:end)=remote_time_cnt0;
        remote_time_cnt  (2:2:end)=remote_time_cnt0; 
        
        temp=reshape([data(4:4:end,135:136)'],1,[]);
        algo_vel_forward_last_pct0  =double(typecast(uint8(temp),'int16')')/32768; 
        algo_vel_forward_last_pct  =[algo_vel_forward_last_pct0; algo_vel_forward_last_pct0];
        algo_vel_forward_last_pct  (1:2:end)=algo_vel_forward_last_pct0;
        algo_vel_forward_last_pct  (2:2:end)=algo_vel_forward_last_pct0;        
        
        temp=reshape([data(3:4:end,215:216)'],1,[]);
        algo_pos_error_00  =double(typecast(uint8(temp),'int16')')/32768*10; 
        algo_pos_error_0  =[algo_pos_error_00; algo_pos_error_00];
        algo_pos_error_0  (1:2:end)=algo_pos_error_00;
        algo_pos_error_0  (2:2:end)=algo_pos_error_00; 
        
        temp=reshape([data(3:4:end,217:218)'],1,[]);
        algo_pos_error_10  =double(typecast(uint8(temp),'int16')')/32768*10; 
        algo_pos_error_1  =[algo_pos_error_10; algo_pos_error_10];
        algo_pos_error_1  (1:2:end)=algo_pos_error_10;
        algo_pos_error_1  (2:2:end)=algo_pos_error_10;  
        
        temp=reshape([data(3:4:end,219:220)'],1,[]);
        algo_pos_error_20  =double(typecast(uint8(temp),'int16')')/32768*10; 
        algo_pos_error_2  =[algo_pos_error_20; algo_pos_error_20];
        algo_pos_error_2  (1:2:end)=algo_pos_error_20;
        algo_pos_error_2  (2:2:end)=algo_pos_error_20;  
        


        
        
%         GSConnected =[GSConnected; GSConnected; GSConnected; GSConnected];
%         GSConnected (1:4:end)=GSConnected;
%         GSConnected (2:4:end)=GSConnected; 
%         GSConnected (3:4:end)=GSConnected; 
%         GSConnected (4:4:end)=GSConnected; 
        
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        temp=reshape([data(2:2:end,165:166)'],1,[]);
        dAB_00=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(2:2:end,167:168)'],1,[]);
        dAB_01=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(2:2:end,169:170)'],1,[]);
        dAB_02=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(2:2:end,171:172)'],1,[]);
        dWB_00=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(2:2:end,173:174)'],1,[]);
        dWB_01=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(2:2:end,175:176)'],1,[]);
        dWB_02=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;         
        temp=reshape([data(1:2:end,153:154)'],1,[]);
        dAB_10=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(1:2:end,155:156)'],1,[]);
        dAB_11=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(1:2:end,157:158)'],1,[]);
        dAB_12=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(1:2:end,159:160)'],1,[]);
        dWB_10=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(1:2:end,161:162)'],1,[]);
        dWB_11=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(1:2:end,163:164)'],1,[]);
        dWB_12=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;         
         temp=reshape([data(1:2:end,165:166)'],1,[]);
        dAB_20=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(1:2:end,167:168)'],1,[]);
        dAB_21=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(1:2:end,169:170)'],1,[]);
        dAB_22=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(1:2:end,171:172)'],1,[]);
        dWB_20=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(1:2:end,173:174)'],1,[]);
        dWB_21=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(1:2:end,175:176)'],1,[]);
        dWB_22=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;           
        temp=reshape([data(2:2:end,159:160)'],1,[]);
        gx_f=double(typecast(uint8(temp),'int16')')/32768*17.5*HD; 
        temp=reshape([data(2:2:end,161:162)'],1,[]);
        gy_f=double(typecast(uint8(temp),'int16')')/32768*-17.5*HD;  
        temp=reshape([data(2:2:end,163:164)'],1,[]);
        gz_f=double(typecast(uint8(temp),'int16')')/32768*-17.5*HD; 
        temp=reshape([data(1:2:end,145:146)'],1,[]);
        ax_f=double(typecast(uint8(temp),'int16')')/32768*80;%%80
        temp=reshape([data(1:2:end,147:148)'],1,[]);
        ay_f=double(typecast(uint8(temp),'int16')')/32768*-80;        
        temp=reshape([data(1:2:end,149:150)'],1,[]);
        az_f=double(typecast(uint8(temp),'int16')')/32768*-80; 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %MAg_YAW
        temp=reshape([data(4:4:end,209:210)'],1,[]);
        rtU_MagYaw_acc_0=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(4:4:end,211:212)'],1,[]);
        rtU_MagYaw_acc_1=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(4:4:end,213:214)'],1,[]);
        rtU_MagYaw_acc_2=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(4:4:end,215:216)'],1,[]);
        rtU_MagYaw_mag_0=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,217:218)'],1,[]);
        rtU_MagYaw_mag_1=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,219:220)'],1,[]);
        rtU_MagYaw_mag_2=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(4:4:end,221:222)'],1,[]);
        rtU_MagYaw_out=double(typecast(uint8(temp),'int16')')/32768*4*HD;
 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %BMS
        temp=reshape([data(2:4:end,113)'],1,[]);
        ASOC=typecast(uint8(temp),'uint8')';
        temp=reshape([data(2:4:end,114)'],1,[]);
        RSOC=typecast(uint8(temp),'uint8')';
        temp=reshape([data(2:4:end,115:116)'],1,[]);
        SOH=typecast(uint8(temp),'uint16')';   
        temp=reshape([data(2:4:end,117:118)'],1,[]);
        Average_Time_To_empty=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,119:120)'],1,[]);
        Cell_Volt_0=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,121:122)'],1,[]);
        Cell_Volt_1=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,123:124)'],1,[]);
        Cell_Volt_2=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,125:126)'],1,[]);
        Cell_Volt_3=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,127:128)'],1,[]);
        Cell_Volt_4=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,129:130)'],1,[]);
        Cell_Volt_5=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,131:132)'],1,[]);
        Cycle_Count=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,133:134)'],1,[]);
        Design_Cap=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,135:136)'],1,[]);
        Design_Volt=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,117:118)'],1,[]);
        FCC=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,119:120)'],1,[]);
        FW_Version=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,121:122)'],1,[]);
        Manufacture_Date=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,123:124)'],1,[]);
        Other_Status=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,125:126)'],1,[]);
        Rem_Cap=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,129:130)'],1,[]);
        Run_Time_TO_Empty=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,131:132)'],1,[]);
        Safty_Status=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:4:end,133:134)'],1,[]);
        Serial_Num=typecast(uint8(temp),'uint16')'; 
        temp=reshape([data(2:2:end,145:146)'],1,[]);
        Temperature=typecast(uint8(temp),'uint16')'; 
        Temperature=Temperature(1:2:end);
        temp=reshape([data(2:2:end,147:148)'],1,[]);
        Total_Cycles=typecast(uint8(temp),'uint16')'; 
        Total_Cycles=Total_Cycles(1:2:end);
        temp=reshape([data(2:2:end,149:150)'],1,[]);
        Voltage=double(typecast(uint8(temp),'uint16')')/1000; 
        Voltage=Voltage(1:2:end);
        temp=reshape([data(2:2:end,151:154)'],1,[]);
        Now_Current=typecast(uint8(temp),'int32')';
        Now_Current=Now_Current(1:2:end);
        temp=reshape([data(2:2:end,155:158)'],1,[]);
        Average_Current=double(typecast(uint8(temp),'int32')')/-1000; 
        Average_Current=Average_Current(1:2:end);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %airspeed
        temp=reshape([data(2:2:end,177:178)'],1,[]);
        diff_press_filtered_pa=double(typecast(uint8(temp),'int16')')/32768*1000;
        temp=reshape([data(2:2:end,179:180)'],1,[]);
        indicated_airspeed=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,181:182)'],1,[]);
        true_airspeed=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,183:184)'],1,[]);
        EAS_Algo=double(typecast(uint8(temp),'int16')')/32768*50;
        temp=reshape([data(2:2:end,185:186)'],1,[]);
        EAS2TAS_Algo=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(2:2:end,189:190)'],1,[]);
        airspeed_used=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(2:2:end,191:192)'],1,[]);
        diff_press_pa_raw=double(typecast(uint8(temp),'int16')')/32768*1000;
        temp=reshape([data(2:2:end,193:194)'],1,[]);
        diff_offset=double(typecast(uint8(temp),'int16')')/32768*300;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        temp=reshape([data(4:4:end,209:212)'],1,[]);
        algo_NAV_lon=double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(4:4:end,213:216)'],1,[]);
        algo_NAV_lat=double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(1:4:end,209:210)'],1,[]);
        algo_NAV_yaw=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(1:4:end,211:212)'],1,[]);
        algo_NAV_pitch=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(1:4:end,213:214)'],1,[]);
        algo_NAV_roll=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(1:4:end,219:220)'],1,[]);
        algo_NAV_alt=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(2:4:end,209:210)'],1,[]);
        algo_NAV_Vn=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(2:4:end,211:212)'],1,[]);
        algo_NAV_Ve=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(2:4:end,213:214)'],1,[]);
        algo_NAV_Vd=double(typecast(uint8(temp),'int16')')/32768*100;   
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DataFusion 
        temp=reshape([data(1:1:end,137:138)'],1,[]);
        Out_initValue_acc_bias0_0=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,139:140)'],1,[]);
        Out_initValue_acc_bias0_1=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,141:142)'],1,[]);
        Out_initValue_acc_bias0_2=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,143:144)'],1,[]);
        Out_initValue_gyro_bias0_0=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,145:146)'],1,[]);
        Out_initValue_gyro_bias0_1=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,147:148)'],1,[]);
        Out_initValue_gyro_bias0_2=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,149:150)'],1,[]);
        Out_initValue_alt0=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,151:152)'],1,[]);
        Out_initValue_lla0_0=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,153:154)'],1,[]);
        Out_initValue_lla0_1=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,155:156)'],1,[]);
        Out_initValue_lla0_2=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,157:158)'],1,[]);
         Out_initValue_gpsvel0_0=double(typecast(uint8(temp),'int16')')/32768*100;  
        temp=reshape([data(1:1:end,159:160)'],1,[]);
        Out_initValue_gpsvel0_1=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,161:162)'],1,[]);
        Out_initValue_gpsvel0_2=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,163:164)'],1,[]);
        Out_initValue_eulerd0_0=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,165:166)'],1,[]);
        Out_initValue_eulerd0_1=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,167:168)'],1,[]);
        Out_initValue_eulerd0_2=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(1:1:end,169:170)'],1,[]);
        Out_initValue_refloc_0=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,171:172)'],1,[]);
        Out_initValue_refloc_1=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,173:174)'],1,[]);
        Out_initValue_refloc_2=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,175:176)'],1,[]);
        Out_initValue_range0 =double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,223:224)'],1,[]);
        stepInfo_step=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,225:226)'],1,[]);
        stepInfo_step_imu=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,227:228)'],1,[]);
        stepInfo_step_mag=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,229:230)'],1,[]);
        stepInfo_step_ublox=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,231:232)'],1,[]);
        stepInfo_step_alt=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,233:234)'],1,[]);
        Out_Sensors_IMU1_time=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,235:236)'],1,[]);
        Out_Sensors_IMU1_accel_x=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,237:238)'],1,[]);
        Out_Sensors_mag2_time=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,239:240)'],1,[]);
        Out_Sensors_mag2_mag_x=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,241:242)'],1,[]);
        Out_Sensors_ublox1_time=double(typecast(uint8(temp),'int16')')/32768*100000;
        temp=reshape([data(1:1:end,243:244)'],1,[]);
        Out_Sensors_ublox1_Lat=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,245:246)'],1,[]);
        Out_Sensors_ublox1_height=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,247:248)'],1,[]);
        Out_Sensors_ublox1_velN=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(1:1:end,249:250)'],1,[]);
        Out_Sensors_ublox1_pDop=double(typecast(uint8(temp),'int16')')/32768*100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %position 
        temp=reshape([data(2:8:end,197:198)'],1,[]);
        mission_item_seq=double(typecast(uint8(temp),'int16')');  
        temp=reshape([data(2:8:end,199:202)'],1,[]);
        mission_item_lat=double(typecast(uint8(temp),'single')');   
        temp=reshape([data(2:8:end,203:206)'],1,[]);
        mission_item_lon=double(typecast(uint8(temp),'single')'); 
        temp=reshape([data(3:8:end,197:198)'],1,[]);
        algo_loc_num=double(typecast(uint8(temp),'int16')');  
        temp=reshape([data(3:8:end,199:202)'],1,[]);
        algo_loc_lat=double(typecast(uint8(temp),'single')')/1e7;   
        temp=reshape([data(3:8:end,203:206)'],1,[]);
        algo_loc_lon=double(typecast(uint8(temp),'single')')/1e7;         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %out taskmode
        temp=reshape([data(16:16:end,229:230)'],1,[]);
        algo_currentPointNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(16:16:end,231:232)'],1,[]);
        algo_prePointNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:16:end,229:230)'],1,[]);
        algo_validPathNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(1:16:end,231:232)'],1,[]);
        algo_headingCmd=double(typecast(uint8(temp),'int16')')/32768*6.3;         
        temp=reshape([data(2:16:end,229:230)'],1,[]);
        algo_distToGo=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(2:16:end,231:232)'],1,[]);
        algo_dz=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(3:16:end,229:230)'],1,[]);
        algo_groundspeedCmd=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(3:16:end,231:232)'],1,[]);
        algo_rollCmd=double(typecast(uint8(temp),'int16')')/32768*10000;           
        temp=reshape([data(4:16:end,229:230)'],1,[]);
        algo_turnRadiusCmd=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(4:16:end,231:232)'],1,[]);
        algo_heightCmd_sl=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(5:16:end,229:232)'],1,[]);
        algo_turnCenterLL_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(6:16:end,229:232)'],1,[]);
        algo_turnCenterLL_1 =double(typecast(uint8(temp),'int32')')*1e-7;                                    
        temp=reshape([data(7:16:end,229:230)'],1,[]);
        algo_dR_turn=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(7:16:end,231)'],1,[]);
        algo_uavMode=double(typecast(uint8(temp),'uint8')'); 
        temp=reshape([data(7:16:end,232)'],1,[]);
        algo_flightTaskMode_sl=double(typecast(uint8(temp),'uint8')'); 
        temp=reshape([data(8:16:end,229)'],1,[]);
        algo_flightControlMode =double(typecast(uint8(temp),'uint8')');    
        temp=reshape([data(8:16:end,230)'],1,[]);
        algo_AutoManualMode =double(typecast(uint8(temp),'uint8')');
        temp=reshape([data(8:16:end,231)'],1,[]);
        algo_comStatus =double(typecast(uint8(temp),'uint8')');
        temp=reshape([data(9:16:end,229:230)'],1,[]);
        algo_maxClimbSpeed_sl=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(10:16:end,229:232)'],1,[]);
        algo_prePathPoint_LLA_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(11:16:end,229:232)'],1,[]);
        algo_prePathPoint_LLA_1=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(12:16:end,229:230)'],1,[]);
        algo_prePathPoint_LLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;  
        temp=reshape([data(13:16:end,229:232)'],1,[]);
        algo_curPathPoint_LLA_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(14:16:end,229:232)'],1,[]);
        algo_curPathPoint_LLA_1=double(typecast(uint8(temp),'int32')')*1e-7;      
        temp=reshape([data(15:16:end,229:230)'],1,[]);
        algo_curPathPoint_LLA_2=double(typecast(uint8(temp),'int16')')/32768*10000; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %out task flight param
        temp=reshape([data(16:16:end,233:236)'],1,[]);
        algo_curHomeLLA_0=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(1:16:end,233:236)'],1,[]);
        algo_curHomeLLA_1=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(2:16:end,233:234)'],1,[]);
        algo_curHomeLLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;
        temp=reshape([data(2:16:end,235:236)'],1,[]);
        algo_curVelNED_0=double(typecast(uint8(temp),'int16')')/32768*50;         
        temp=reshape([data(3:16:end,233:234)'],1,[]);
        algo_curVelNED_1=double(typecast(uint8(temp),'int16')')/32768*50;  
        temp=reshape([data(3:16:end,235:236)'],1,[]);
        algo_curVelNED_2=double(typecast(uint8(temp),'int16')')/32768*30;         
        temp=reshape([data(4:16:end,233:234)'],1,[]);
        algo_curSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;  
        temp=reshape([data(4:16:end,235:236)'],1,[]);
        algo_curAirSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(5:16:end,233:234)'],1,[]);
        algo_curEuler_0=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(5:16:end,235:236)'],1,[]);
        algo_curEuler_1=double(typecast(uint8(temp),'int16')')/32768*10;         
        temp=reshape([data(6:16:end,233:234)'],1,[]);
        algo_curEuler_2=double(typecast(uint8(temp),'int16')')/32768*10;       
        temp=reshape([data(6:16:end,235:236)'],1,[]);
        algo_curWB_0=double(typecast(uint8(temp),'int16')')/32768*10;         
        temp=reshape([data(7:16:end,233:234)'],1,[]);
        algo_curWB_1=double(typecast(uint8(temp),'int16')')/32768*10000;       
        temp=reshape([data(7:16:end,235:236)'],1,[]);
        algo_curWB_2=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(8:16:end,233:234)'],1,[]);
        algo_curPosNED_0=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(8:16:end,235:236)'],1,[]);
        algo_curPosNED_1=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(9:16:end,233:234)'],1,[]);
        algo_curPosNED_2=double(typecast(uint8(temp),'int16')')/32768*10000;        
        temp=reshape([data(10:16:end,233:236)'],1,[]);
        algo_curLLA_0=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(11:16:end,233:236)'],1,[]);
        algo_curLLA_1=double(typecast(uint8(temp),'int32')')*1e-7;  
        temp=reshape([data(12:16:end,233:234)'],1,[]);
        algo_curLLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(12:16:end,235:236)'],1,[]);
        algo_curGroundSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;       
        temp=reshape([data(13:16:end,233:234)'],1,[]);
        algo_curAccZ=double(typecast(uint8(temp),'int16')')/32768*10000;   
          

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        len=length(algo_yaw);
        algo_yaw1=algo_yaw;
        sq=50;
        for i=1:len-1
            if((algo_yaw1(i+1)-algo_yaw1(i))>sq)
                algo_yaw1(i+1:end)=algo_yaw1(i+1:end)-360;
            elseif((algo_yaw1(i+1)-algo_yaw1(i))<-sq)
                algo_yaw1(i+1:end)=algo_yaw1(i+1:end)+360;
            end  
        end
        %IMU_tmp
        temp=reshape([data(8:8:end,197:198)'],1,[]);
        imu_tmp=double(typecast(uint8(temp),'int16')')/32768*100;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        time_imu=save_time;
        time_baro=save_time(1:2:end);
        time_mag=save_time(1:4:end);
        time_radar=save_time(1:4:end);
        time_ublox=save_time(1:4:end);
        time_algo=save_time(1:2:end);
        time_bms=save_time(1:4:end);
        time_arspeed=save_time(1:2:end);
        time_mag_yaw=save_time(1:4:end);
        time_sl_att=save_time(1:4:end);
        time_sl_DataFusion=save_time(1:end);
        time_imu_tmp=save_time(1:8:end);
        time_pos_tmp=save_time(1:8:end);
        time_sl_taskmode=save_time(1:16:end);
        time_sl_task_flight=save_time(1:16:end);
               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %IMU
%         len=100;
%         ax(1:len)=[];
%         ay(1:len)=[];  
%         az(1:len)=[];
%         gx(1:len)=[];
%         gy(1:len)=[];
%         gz(1:len)=[];
%         time_imu(1:len)=[];
%         
%         diff_time=diff(time_imu);
%         time_imu(diff_time==0)=[];
%         ax(diff_time==0)=[];
%         ay(diff_time==0)=[];
%         az(diff_time==0)=[];
%         gx(diff_time==0)=[];
%         gy(diff_time==0)=[];
%         gz(diff_time==0)=[];
%         
%         
        data_imu_txt={ 
        'time_imu'
        'ax';
        'ay';
        'az';      
        'gx';     
        'gy';   
        'gz'; 
        };
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %baro
        len=500;
        time_baro(1:len)=[];
        altitue(1:len)=[];  
        temperature(1:len)=[];
        pressure(1:len)=[];
        temperature_gs(1:len)=[];
        pressure_gs(1:len)=[];
        data_baro_txt={
        'time_baro'
        'altitue';        
        'temperature';        
        'pressure';             
        'temperature_gs';           
        'pressure_gs';  
        };
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Mag
    data_mag_txt={
        'time_mag';
        'mag1_x';
        'mag1_y';
        'mag1_z';
        'mag2_x';
        'mag2_y';
        'mag2_z';        
        'mag3_x';
        'mag3_y';
        'mag3_z';
        'mag1_xx';
        'mag1_yx';
        'mag1_zx'; 
        'mag2_xx';
        'mag2_yx';
        'mag2_zx';
        };
%     yaw_mag1=atan2d(mag1_z,mag1_x);
%     yaw_mag2=atan2d(mag2_z,mag2_x);
%     plot(yaw_mag1+55);
%     hold on
%     plot(yaw_mag1)
%     plot(yaw_mag2)
  
    
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %radar
        data_radar_txt={
        'time_radar'
        'radar_SNR';
        'radar_Flag';
        'radar_Range';
        };
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %ublox
        ublox_cod=atan2d(ublox_velE,ublox_velN);
   data_ublox_txt={
        'time_ublox'
        'ublox_iTOW';
        'ublox_velE';
        'ublox_velN';       
        'ublox_velD';         
        'ublox_lon';  
        'ublox_lat';    
        'ublox_height';   
        'ublox_pDOP'; 
        'ublox_numSV';
        'ublox_V';
        'ublox_theta';
        'ublox_cod';
        };
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %BMS
   data_bms_txt={
        'time_bms'      
        'ASOC';
        'RSOC';
        'SOH';
        'Average_Time_To_empty';
        'Cell_Volt_0';
        'Cell_Volt_1';
        'Cell_Volt_2';
        'Cell_Volt_3';
        'Cell_Volt_4';
        'Cell_Volt_5';
        'Cycle_Count';
        'Design_Cap';
        'Design_Volt';
        'FCC';
        'FW_Version';
        'Manufacture_Date';
        'Other_Status';
        'Rem_Cap';
        'Run_Time_TO_Empty';
        'Safty_Status';
        'Serial_Num';
        'Temperature';
        'Total_Cycles';
        'Voltage';
        'Now_Current';
        'Average_Current';
  };
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %algo
        data_ck_txt={ 
        'time_algo'
        'Switch_LockStatus';         
        'Lock_Status';    
        'algo_mode';  
        'PLANE_COPTER_MODE';          
        'src_pitch';   
        'src_roll'; 
        'src_throttle';         
        'src_yaw';     
        'algo_remote_ct_st_pitch'; 
        'algo_remote_ct_st_roll';           
        'algo_remote_ct_st_throttle';           
        'algo_remote_ct_st_yaw';           
        'algo_roll';           
        'algo_pitch';           
        'algo_yaw';           
        'algo_curr_vel_0';           
        'algo_curr_vel_1';
        'algo_curr_vel_2'; 
        'algo_curr_pos_0';
        'algo_curr_pos_1';
%         'algo_curr_pos_2';
        'algo_curr_alt';
        'algo_rate_target_ang_vel_0';
        'algo_rate_target_ang_vel_1';
        'algo_rate_target_ang_vel_2';
        'algo_roll_in';
        'algo_pitch_in';
        'algo_yaw_in';
        'algo_throttle_in';
        'algo_attitude_target_euler_rate_0';
        'algo_attitude_target_euler_rate_1';
        'algo_attitude_target_euler_rate_2';
        'algo_attitude_target_euler_angle_0';
        'algo_attitude_target_euler_angle_1';
        'algo_attitude_target_euler_angle_2';
        'algo_pwm_out_1';
        'algo_pwm_out_2';
        'algo_pwm_out_3';
        'algo_pwm_out_4';
        'algo_attitude_error_vector_0';
        'algo_attitude_error_vector_1';
        'algo_attitude_error_vector_2';
        'algo_speed_out_0';
        'algo_speed_out_1';
        'algo_speed_out_2';
        'algo_speed_out_3';
        'algo_pos_target_0';
        'algo_pos_target_1';
        'algo_pos_target_2';
        'algo_vel_target_0';
        'algo_vel_target_1';
        'algo_vel_target_2';
        'algo_accel_target_0';
        'algo_accel_target_1';
        'algo_accel_target_2';
        'algo_z_accel_meas';
        'algo_climb_rate_cms';
        'algo_throttle_filter';
        'algo_vel_desired_2';
        'algo_throttle_lower';
        'algo_throttle_upper';
        'algo_limit_pos_up';
        'algo_nav_pitch_cd';        
        'algo_latAccDem';
        'algo_yaw_out'; 
        'algo_throttle_dem';         
        'algo_k_rudder';         
        'algo_k_elevator';
        'algo_k_throttle'; 
        'algo_k_aileron'; 
        'ax_f';
        'ay_f';
        'az_f';
        'gx_f';
        'gy_f';
        'gz_f';   
        'algo_servo_out_0';
        'algo_servo_out_1';
        'algo_servo_out_2';
        'algo_servo_out_3';
        'algo_pitch_dem';
        'algo_hgt_dem';
        'algo_throttle_dem';
        'algo_arspeed';
        'algo_roll_target_pilot';
        'algo_pitch_target_pilot';
        'algo_current_loc_0 ';
        'algo_current_loc_1 ';
        'algo_tail_tilt';
        'algo_tail_tilt_pwm';
        'algo_nav_roll';
        'algo_nav_pitch';
        'algo_roll_target';
        'algo_pitch_target';
        'algo_weathervane_last_output';
        'algo_heightCmd';
        'algo_maxClimbSpeed';
        'algo_flightTaskMode';
        'GSConnected';
        'Remote_Disconnect_Status';
        'remote_time_cnt';
        'algo_vel_forward_last_pct';
        'FC_VERSION';
        'algo_pos_error_0';
        'algo_pos_error_1';
        'algo_pos_error_2';
       };
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %arspeed
    data_arspeed_txt={
        'time_arspeed';
        'diff_press_filtered_pa';
        'indicated_airspeed';
        'true_airspeed';
        'EAS_Algo';
        'EAS2TAS_Algo';
        'airspeed_used';
        'diff_press_pa_raw';
        'diff_offset';
        };
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %MAg_YAW
         data_mag_yaw_txt={
        'time_mag_yaw'
        'rtU_MagYaw_acc_0';
        'rtU_MagYaw_acc_1';
        'rtU_MagYaw_acc_2';
        'rtU_MagYaw_mag_0';
        'rtU_MagYaw_mag_1';
        'rtU_MagYaw_mag_2';
        'rtU_MagYaw_out';
        };
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     data_sl_att_txt={
        'time_sl_att'
        'algo_NAV_lon';
        'algo_NAV_lat';
        'algo_NAV_alt';
        'algo_NAV_yaw'; 
        'algo_NAV_pitch'; 
        'algo_NAV_roll'; 
        'algo_NAV_Vn';
        'algo_NAV_Ve';
        'algo_NAV_Vd';  
        };
    %DataFusion 
     data_sl_DataFusion_txt={     
        'time_sl_DataFusion';
        'Out_initValue_acc_bias0_0';  
        'Out_initValue_acc_bias0_1';
        'Out_initValue_acc_bias0_2';
        'Out_initValue_gyro_bias0_0';
        'Out_initValue_gyro_bias0_1';
        'Out_initValue_gyro_bias0_2';
        'Out_initValue_alt0';
        'Out_initValue_lla0_0';
        'Out_initValue_lla0_1';
        'Out_initValue_lla0_2';
        'Out_initValue_gpsvel0_0';
        'Out_initValue_gpsvel0_1';
        'Out_initValue_gpsvel0_2';
        'Out_initValue_eulerd0_0';
        'Out_initValue_eulerd0_1';
        'Out_initValue_eulerd0_2';
        'Out_initValue_refloc_0';
        'Out_initValue_refloc_1';
        'Out_initValue_refloc_2';
        'Out_initValue_range0';
        'stepInfo_step';
        'stepInfo_step_imu';
        'stepInfo_step_mag';
        'stepInfo_step_ublox';
        'stepInfo_step_alt';
        'Out_Sensors_IMU1_time';
        'Out_Sensors_IMU1_accel_x';
        'Out_Sensors_mag2_time';
        'Out_Sensors_mag2_mag_x';
        'Out_Sensors_ublox1_time';
        'Out_Sensors_ublox1_Lat';
        'Out_Sensors_ublox1_height';
        'Out_Sensors_ublox1_velN';
        'Out_Sensors_ublox1_pDop';
        };
    
    data_imu_tmp_txt={  
    'time_imu_tmp';
    'imu_tmp';
    };
    data_position_txt={
        'time_pos_tmp';
        'mission_item_seq';  
        'mission_item_lat';   
        'mission_item_lon'; 
        'algo_loc_num';  
        'algo_loc_lat';   
        'algo_loc_lon';
        };
%     data_sl_temp={
%         'time_sl_temp'
%         'SystemHealthStatus';
%         'PayLoad';
%         'AutoManualMode';   
%         'Task'
%     };
    data_sl_taskmode={
        'time_sl_taskmode';
        'algo_currentPointNum';    
       'algo_prePointNum';        
       'algo_validPathNum';       
       'algo_headingCmd';          
       'algo_distToGo';            
       'algo_dz';                 
       'algo_groundspeedCmd';      
       'algo_rollCmd';           
       'algo_turnRadiusCmd';       
       'algo_heightCmd_sl';           
       'algo_turnCenterLL_0';     
       'algo_turnCenterLL_1';    
       'algo_dR_turn';             
       'algo_uavMode';             
       'algo_flightTaskMode_sl';      
       'algo_flightControlMode';   
       'algo_AutoManualMode';      
       'algo_comStatus';           
       'algo_maxClimbSpeed_sl';       
       'algo_prePathPoint_LLA_0'; 
       'algo_prePathPoint_LLA_1'; 
       'algo_prePathPoint_LLA_2'; 
       'algo_curPathPoint_LLA_0'; 
       'algo_curPathPoint_LLA_1'; 
       'algo_curPathPoint_LLA_2';
    };
    data_sl_task_flight={
       'time_sl_task_flight';
       'algo_curHomeLLA_0';
       'algo_curHomeLLA_1';
       'algo_curHomeLLA_2';
       'algo_curVelNED_0'; 
       'algo_curVelNED_1'; 
       'algo_curVelNED_2'; 
       'algo_curSpeed';     
       'algo_curAirSpeed';  
       'algo_curEuler_0';  
       'algo_curEuler_1';  
       'algo_curEuler_2';  
       'algo_curWB_0';     
       'algo_curWB_1';     
       'algo_curWB_2';     
       'algo_curPosNED_0'; 
       'algo_curPosNED_1'; 
       'algo_curPosNED_2'; 
       'algo_curLLA_0';    
       'algo_curLLA_1';    
       'algo_curLLA_2';    
       'algo_curGroundSpeed';    
       'algo_curAccZ';           
    };

  % --------------------------------------------------algo-------------------
