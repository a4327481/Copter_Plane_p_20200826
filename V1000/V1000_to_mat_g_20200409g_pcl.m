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
BLOCK_SIZE=296;

finf = dir([PathName,'\\*.bin']);
len_finf = length(finf);
cell_name={finf.name};
nat_name=natsort(cell_name);

for j=1:len_finf
    filename = [PathName,'\\',nat_name{j}];  %%构造第k个文件的位置（合并文件路径和文件名）
    fp=fopen(filename,'r');
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
%         indexn=find(mod(Count,8)==1);
%         if( indexn(1) >1 )
%             data=data(indexn(1):end,:);
%         end
               
        [m,mm]=size(data);
        m=floor(m/16)*16;
        data=data(1:m,:);
%         datacolumn=data;
        Count=binDecode(data,1,0,0); 
        HD=180/pi;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        index_20=find(mod(Count,2)==0);
        index_21=find(mod(Count,2)==1);
        
        index_40=find(mod(Count,4)==0);
        index_41=find(mod(Count,4)==1);
        index_42=find(mod(Count,4)==2);
        index_43=find(mod(Count,4)==3);
        
        index_80=find(mod(Count,8)==0);
        index_81=find(mod(Count,8)==1);
        index_82=find(mod(Count,8)==2);
        index_83=find(mod(Count,8)==3);
        index_84=find(mod(Count,8)==4);
        index_85=find(mod(Count,8)==5);
        index_86=find(mod(Count,8)==6);
        index_87=find(mod(Count,8)==7);   
        
        index_160=find(mod(Count,16)==0);
        index_161=find(mod(Count,16)==1);
        index_162=find(mod(Count,16)==2);
        index_163=find(mod(Count,16)==3);
        index_164=find(mod(Count,16)==4);
        index_165=find(mod(Count,16)==5);
        index_166=find(mod(Count,16)==6);
        index_167=find(mod(Count,16)==7);   
        index_168=find(mod(Count,16)==8);
        index_169=find(mod(Count,16)==9);
        index_1610=find(mod(Count,16)==10);
        index_1611=find(mod(Count,16)==11);
        index_1612=find(mod(Count,16)==12);
        index_1613=find(mod(Count,16)==13);
        index_1614=find(mod(Count,16)==14);
        index_1615=find(mod(Count,16)==15);   
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SYSTEM
        
        temp=reshape([data(:,251:254)'],1,[]);
        save_time=double(typecast(uint8(temp),'uint32')')*1e-3;
        
%         temp=reshape([data(8:8:end,255:256)'],1,[]);        
%         FC_VERSION0=(typecast(uint8(temp),'uint16'))';
%         FC_VERSION  =[FC_VERSION0; FC_VERSION0;FC_VERSION0;FC_VERSION0];
%         FC_VERSION  (1:4:end)=FC_VERSION0;
%         FC_VERSION  (2:4:end)=FC_VERSION0;   
%         FC_VERSION  (3:4:end)=FC_VERSION0;
%         FC_VERSION  (4:4:end)=FC_VERSION0;
%         temp=reshape([data(4:16:end,243:246)'],1,[]);        
%         fc_sn_num=(typecast(uint8(temp),'uint32'))';  
%         
%         temp = reshape([data(find(mod(Count,1)==0),1:2)'],1,[]);
%         save_count=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
%         TempName.save_count = save_count; % create struct
%         temp = reshape([data(find(mod(Count,1)==0),251:254)'],1,[]);
%         save_time=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
%         TempName.save_time = save_time; % create struct
        temp = reshape([data(find(mod(Count,8)==0),255:256)'],1,[]);
        FCVERSION=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
        temp = reshape([data(find(mod(Count,16)==4),243:246)'],1,[]);
        fc_sn_num=double(typecast(uint8(temp),'uint32')');
        fc_sn_num_hex=dec2hex(fc_sn_num);
        temp = reshape([data(find(mod(Count,16)==10),291:291)'],1,[]);
        algo_plane_mode=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
        temp = reshape([data(find(mod(Count,8)==0),207:208)'],1,[]);
        lose_connectgs_cntms= double(typecast(uint8(temp),'uint16')')/1*1.0000000000;

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
        temp=reshape([data(index_21,41:42)'],1,[]);
        altitue=double(typecast(uint8(temp),'int16')')/32768*2000;        
        temp=reshape([data(index_21,43:44)'],1,[]);
        temperature=double(typecast(uint8(temp),'int16')')/32768*100;        
        temp=reshape([data(index_21,45:46)'],1,[]);
        pressure=double(typecast(uint8(temp),'int16')')/32768*1100;             
        temp=reshape([data(index_21,47:48)'],1,[]);
        temperature_gs=double(typecast(uint8(temp),'int16')')/32768*1100;           
        temp=reshape([data(index_21,49:50)'],1,[]);
        pressure_gs=double(typecast(uint8(temp),'int16')')/32768*1100;  
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Mag

        % mag1
        temp=reshape([data(index_40,109:110)'],1,[]);
        mag1_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,111:112)'],1,[]);
        mag1_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,113:114)'],1,[]);
        mag1_z=double(typecast(uint8(temp),'int16')')/32768*2;  
        temp=mag1_x;
        mag1_x=-mag1_y;
        mag1_y=-temp;
        mag1_z=-mag1_z;
        % mag2
        temp=reshape([data(index_40,115:116)'],1,[]);
        mag2_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,117:118)'],1,[]);
        mag2_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,119:120)'],1,[]);
        mag2_z=double(typecast(uint8(temp),'int16')')/32768*2;    
        
        temp=mag2_x;
        mag2_x=-mag2_y;
        mag2_y=-temp;
        mag2_z=-mag2_z;
        % mag3
        temp=reshape([data(index_40,121:122)'],1,[]);
        mag3_x=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,123:124)'],1,[]);
        mag3_y=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,125:126)'],1,[]);
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
        index_21=find(mod(Count,2)==1);
        temp=reshape([data(index_21,191:192)'],1,[]);
        mag2_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(index_21,193:194)'],1,[]);
        mag2_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(index_21,195:196)'],1,[]);
        mag2_zx=double(typecast(uint8(temp),'int16')')/32768*2;
        mag2_xx=mag2_xx(1:2:end);
        mag2_yx=mag2_yx(1:2:end);
        mag2_zx=mag2_zx(1:2:end);
        temp=mag2_xx;
        mag2_xx=-mag2_yx;
        mag2_yx=-temp;
        mag2_zx=-mag2_zx;
        
        temp=reshape([data(index_21,167:168)'],1,[]);
        mag1_xx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(index_21,169:170)'],1,[]);
        mag1_yx=double(typecast(uint8(temp),'int16')')/32768*2; 
        temp=reshape([data(index_21,171:172)'],1,[]);
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
        temp=reshape([data(index_43,117)'],1,[]);
        radar_SNR=uint8(temp');
        temp=reshape([data(index_43,118)'],1,[]);
        radar_Flag=uint8(temp');
        temp=reshape([data(index_43,119:120)'],1,[]);
        radar_Range=double(typecast(uint8(temp),'int16')')/32768*2000;
        radar_Range=double(typecast(uint8(temp),'int16')')/100;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %ublox
        ublox_index=find(mod(Count,4)==1);
        ublox_index_40=find(mod(Count,4)==0);
        
        temp=reshape([data(index_41,109:112)'],1,[]);
        ublox_iTOW=double(typecast(uint8(temp),'int32')');
        temp=reshape([data(index_41,113:116)'],1,[]);
        ublox_velE=double(typecast(uint8(temp),'int32')')*1e-3;
        temp=reshape([data(index_41,117:120)'],1,[]);
        ublox_velN=double(typecast(uint8(temp),'int32')')*1e-3;       
        temp=reshape([data(index_41,121:124)'],1,[]);
        ublox_velD=double(typecast(uint8(temp),'int32')')*1e-3;         
        temp=reshape([data(index_41,125:128)'],1,[]);
        ublox_lon=double(typecast(uint8(temp),'int32')')*1e-7;  
        temp=reshape([data(index_41,129:132)'],1,[]);
        ublox_lat=double(typecast(uint8(temp),'int32')')*1e-7;    
        temp=reshape([data(index_41,133:136)'],1,[]);
        ublox_height=double(typecast(uint8(temp),'int32')')*1e-3;   
        temp=reshape([data(ublox_index_40,127:128)'],1,[]);
        ublox_pDOP=double(typecast(uint8(temp),'uint16')')*1e-2; 
        temp=reshape([data(ublox_index_40,129)'],1,[]);
        ublox_numSV=typecast(uint8(temp),'uint8')';
        ublox_V=hypot(ublox_velN,ublox_velE);
        ublox_theta=-atan2d(ublox_velD,ublox_V);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %algo

        temp=reshape([data(index_20,41)'],1,[]);
        Switch_LockStatus=typecast(uint8(temp),'uint8')';         
        temp=reshape([data(index_20,42)'],1,[]);
        Lock_Status=typecast(uint8(temp),'uint8')';    
        temp=reshape([data(index_20,43)'],1,[]);
        algo_mode=typecast(uint8(temp),'uint8')';  
        temp=reshape([data(index_20,44)'],1,[]);
        PLANE_COPTER_MODE=typecast(uint8(temp),'uint8')'; 
        temp=reshape([data(index_20,45:46)'],1,[]);
        src_tilt=double(typecast(uint8(temp),'uint16')');           
        temp=reshape([data(index_20,47:48)'],1,[]);
        src_pitch=double(typecast(uint8(temp),'uint16')');   
        temp=reshape([data(index_20,49:50)'],1,[]);
        src_roll=double(typecast(uint8(temp),'uint16')'); 
        temp=reshape([data(index_20,51:52)'],1,[]);
        src_throttle=double(typecast(uint8(temp),'uint16')');         
        temp=reshape([data(index_20,53:54)'],1,[]);
        src_yaw=double(typecast(uint8(temp),'uint16')');     
        temp=reshape([data(index_20,55:56)'],1,[]);
        algo_remote_ct_st_pitch=double(typecast(uint8(temp),'int16')')/32768*80; 
        temp=reshape([data(index_20,57:58)'],1,[]);
        algo_remote_ct_st_roll=double(typecast(uint8(temp),'int16')')/32768*80;           
        temp=reshape([data(index_20,59:60)'],1,[]);
        algo_remote_ct_st_throttle=double(typecast(uint8(temp),'int16')')/32768*400;           
        temp=reshape([data(index_20,61:62)'],1,[]);
        algo_remote_ct_st_yaw=double(typecast(uint8(temp),'int16')')/32768*200;           
        temp=reshape([data(index_20,63:64)'],1,[]);
        algo_roll=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(index_20,65:66)'],1,[]);
        algo_pitch=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(index_20,67:68)'],1,[]);
        algo_yaw=double(typecast(uint8(temp),'int16')')/32768*4*HD;           
        temp=reshape([data(index_20,69:70)'],1,[]);
        algo_curr_vel_0=double(typecast(uint8(temp),'int16')')/32768*30;           
        temp=reshape([data(index_20,71:72)'],1,[]);
        algo_curr_vel_1=double(typecast(uint8(temp),'int16')')/32768*30;
        temp=reshape([data(index_20,73:74)'],1,[]);
        algo_curr_vel_2=double(typecast(uint8(temp),'int16')')/32768*30; 
        temp=reshape([data(index_20,75:76)'],1,[]);
        algo_curr_pos_0=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,77:78)'],1,[]);
        algo_curr_pos_1=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,79:80)'],1,[]);
        algo_curr_pos_2=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,81:82)'],1,[]);
        algo_curr_alt=double(typecast(uint8(temp),'int16')')/32768*10000*10/100;%%%%%%%%%%%%%%%%20200215
        temp=reshape([data(index_20,83:84)'],1,[]);
        algo_rate_target_ang_vel_0=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(index_20,85:86)'],1,[]);
        algo_rate_target_ang_vel_1=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(index_20,87:88)'],1,[]);
        algo_rate_target_ang_vel_2=double(typecast(uint8(temp),'int16')')/32768*500*HD;
        temp=reshape([data(index_20,89:90)'],1,[]);
        algo_roll_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(index_20,91:92)'],1,[]);
        algo_pitch_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(index_20,93:94)'],1,[]);
        algo_yaw_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(index_20,95:96)'],1,[]);
        algo_throttle_in=double(typecast(uint8(temp),'int16')')/32768*1.1;
        temp=reshape([data(index_20,97:98)'],1,[]);
        algo_attitude_target_euler_rate_0=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(index_20,99:100)'],1,[]);
        algo_attitude_target_euler_rate_1=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(index_20,101:102)'],1,[]);
        algo_attitude_target_euler_rate_2=double(typecast(uint8(temp),'int16')')/32768*6.3;
        temp=reshape([data(index_20,103:104)'],1,[]);
        algo_attitude_target_euler_angle_0=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(index_20,105:106)'],1,[]);
        algo_attitude_target_euler_angle_1=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(index_20,107:108)'],1,[]);
        algo_attitude_target_euler_angle_2=double(typecast(uint8(temp),'int16')')/32768*3.15*HD;
        temp=reshape([data(index_21,51:52)'],1,[]);
        algo_pwm_out_1=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(index_21,53:54)'],1,[]);
        algo_pwm_out_2=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(index_21,55:56)'],1,[]);
        algo_pwm_out_3=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(index_21,57:58)'],1,[]);
        algo_pwm_out_4=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(index_21,59:60)'],1,[]);
        algo_attitude_error_vector_0=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(index_21,61:62)'],1,[]);
        algo_attitude_error_vector_1=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(index_21,63:64)'],1,[]);
        algo_attitude_error_vector_2=double(typecast(uint8(temp),'int16')')/32768*3.15;
        temp=reshape([data(index_21,65:66)'],1,[]);
        algo_speed_out_0=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(index_21,67:68)'],1,[]);
        algo_speed_out_1=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(index_21,69:70)'],1,[]);
        algo_speed_out_2=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(index_21,71:72)'],1,[]);
        algo_speed_out_3=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(index_21,73:74)'],1,[]);
        algo_pos_target_0=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_21,75:76)'],1,[]);
        algo_pos_target_1=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_21,77:78)'],1,[]);
        algo_pos_target_2=double(typecast(uint8(temp),'int16')')/32768*100*10;%%20200215
        temp=reshape([data(index_21,79:80)'],1,[]);
        algo_vel_target_0=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(index_21,81:82)'],1,[]);
        algo_vel_target_1=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(index_21,83:84)'],1,[]);
        algo_vel_target_2=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(index_21,85:86)'],1,[]);
        algo_accel_target_0=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_21,87:88)'],1,[]);
        algo_accel_target_1=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_21,89:90)'],1,[]);
        algo_accel_target_2=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_21,91:92)'],1,[]);
        algo_z_accel_meas=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_21,93:94)'],1,[]);
        algo_climb_rate_cms=double(typecast(uint8(temp),'int16')')/32768*5;
        temp=reshape([data(index_21,95:96)'],1,[]);
        algo_throttle_filter=double(typecast(uint8(temp),'int16')')/32768*20;
        temp=reshape([data(index_21,97:98)'],1,[]);
        algo_vel_desired_2=double(typecast(uint8(temp),'int16')')/32768*10; 
        temp=reshape([data(index_21,99)'],1,[]);
        temp1=typecast(uint8(temp),'uint8');
        algo_throttle_lower=bitand(temp1,1)/1;
        algo_throttle_upper=bitand(temp1,2^1)/2^1;
        algo_limit_pos_up=bitand(temp1,2^2)/2^2;
        temp=reshape([data(index_21,101:102)'],1,[]);
        algo_nav_pitch_cd=double(typecast(uint8(temp),'int16')')/32768*3600;        
        temp=reshape([data(index_21,103:104)'],1,[]);
        algo_latAccDem=double(typecast(uint8(temp),'int16')')/32768*20;
        temp=reshape([data(index_21,105:106)'],1,[]);
        algo_yaw_out=double(typecast(uint8(temp),'int16')')/32768*4500; 
        temp=reshape([data(index_21,107:108)'],1,[]);
        algo_throttle_dem=double(typecast(uint8(temp),'int16')')/32768*1;         
        temp=reshape([data(index_20,137:138)'],1,[]);
        algo_k_rudder=double(typecast(uint8(temp),'int16')')/32768*45;         
        temp=reshape([data(index_20,139:140)'],1,[]);
        algo_k_elevator=double(typecast(uint8(temp),'int16')')/32768*45;
        temp=reshape([data(index_20,141:142)'],1,[]);
        algo_k_throttle=double(typecast(uint8(temp),'int16')')/32768*10; 
        temp=reshape([data(index_20,143:144)'],1,[]);
        algo_k_aileron=double(typecast(uint8(temp),'int16')')/32768*45; 
        temp=reshape([data(index_21,137:138)'],1,[]);
        algo_servo_out_0=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_21,139:140)'],1,[]);
        algo_servo_out_1=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_21,141:142)'],1,[]);
        algo_servo_out_2=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_21,143:144)'],1,[]);
        algo_servo_out_3=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_21,177:178)'],1,[]);
        algo_pitch_dem=double(typecast(uint8(temp),'int16')')/32768*3.15*HD; 
        temp=reshape([data(index_21,179:180)'],1,[]);
        algo_hgt_dem=double(typecast(uint8(temp),'int16')')/32768*1000; 
        temp=reshape([data(index_21,181:182)'],1,[]);
        algo_throttle_dem=double(typecast(uint8(temp),'int16')')/32768*1.1; 
        temp=reshape([data(index_21,183:184)'],1,[]);
        algo_arspeed=double(typecast(uint8(temp),'int16')')/32768*50; 
        temp=reshape([data(index_20,195:196)'],1,[]);
        algo_roll_target_pilot =double(typecast(uint8(temp),'int16')')/32768*50;        
        temp=reshape([data(index_21,185:186)'],1,[]);
        algo_pitch_target_pilot=double(typecast(uint8(temp),'int16')')/32768*50;
        temp=reshape([data(index_20,165:168)'],1,[]);
        algo_current_loc_0 =double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(index_20,169:172)'],1,[]);
        algo_current_loc_1 =double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(index_20,79:80)'],1,[]);
        algo_tail_tilt =double(typecast(uint8(temp),'int16')')/100; 
        temp=reshape([data(index_21,165:166)'],1,[]);
        algo_tail_tilt_pwm =double(typecast(uint8(temp),'int16')');     
        
        algo_nav_roll=cosd(algo_pitch).*atan(algo_latAccDem * 0.101972) *HD;
        algo_nav_pitch=algo_nav_pitch_cd/100;
        temp=reshape([data(index_21,151:152)'],1,[]);
        algo_roll_target=double(typecast(uint8(temp),'int16')')/32768*50; 
        temp=reshape([data(index_21,153:154)'],1,[]);
        algo_pitch_target=double(typecast(uint8(temp),'int16')')/32768*50;  
        temp=reshape([data(index_20,175:176)'],1,[]);
        algo_weathervane_last_output=double(typecast(uint8(temp),'int16')')/32768*1;         
        temp=reshape([data(index_40,220)'],1,[]);
        algo_PathMode0=double(typecast(uint8(temp),'uint8')'); 
        algo_PathMode=[algo_PathMode0; algo_PathMode0];
        algo_PathMode(1:2:end)=algo_PathMode0;
        algo_PathMode(2:2:end)=algo_PathMode0;
        temp=reshape([data(index_41,221:222)'],1,[]);
        algo_heightCmd0 =double(typecast(uint8(temp),'int16')')/32768*300; 
        algo_heightCmd =[algo_heightCmd0; algo_heightCmd0];
        algo_heightCmd (1:2:end)=algo_heightCmd0;
        algo_heightCmd (2:2:end)=algo_heightCmd0;
        temp=reshape([data(index_42,221:222)'],1,[]);
        algo_maxClimbSpeed0 =double(typecast(uint8(temp),'int16')')/32768*500; 
        algo_maxClimbSpeed =[algo_maxClimbSpeed0; algo_maxClimbSpeed0];
        algo_maxClimbSpeed (1:2:end)=algo_maxClimbSpeed0;
        algo_maxClimbSpeed (2:2:end)=algo_maxClimbSpeed0;
        temp=reshape([data(index_43,209)'],1,[]);
        algo_flightTaskMode0 =double(typecast(uint8(temp),'uint8')'); 
        algo_flightTaskMode =[algo_flightTaskMode0; algo_flightTaskMode0];
        algo_flightTaskMode (1:2:end)=algo_flightTaskMode0;
        algo_flightTaskMode (2:2:end)=algo_flightTaskMode0;    
        
        temp=reshape([data(index_80,205)'],1,[]);
        GSConnected0 =double(typecast(uint8(temp),'uint8')'); 
        GSConnected =[GSConnected0; GSConnected0; GSConnected0; GSConnected0];
        GSConnected (1:4:end)=GSConnected0;
        GSConnected (2:4:end)=GSConnected0; 
        GSConnected (3:4:end)=GSConnected0; 
        GSConnected (4:4:end)=GSConnected0; 

        temp=reshape([data(index_21,187)'],1,[]);
        Remote_Disconnect_Status =double(typecast(uint8(temp),'uint8')'); 
 
        
        temp=reshape([data(index_40,135:136)'],1,[]);
        remote_time_cnt0  =double(typecast(uint8(temp),'int16')'); 
        remote_time_cnt  =[remote_time_cnt0; remote_time_cnt0];
        remote_time_cnt  (1:2:end)=remote_time_cnt0;
        remote_time_cnt  (2:2:end)=remote_time_cnt0; 
        
        temp=reshape([data(index_40,135:136)'],1,[]);
        algo_vel_forward_last_pct0  =double(typecast(uint8(temp),'int16')')/32768; 
        algo_vel_forward_last_pct  =[algo_vel_forward_last_pct0; algo_vel_forward_last_pct0];
        algo_vel_forward_last_pct  (1:2:end)=algo_vel_forward_last_pct0;
        algo_vel_forward_last_pct  (2:2:end)=algo_vel_forward_last_pct0;        
        
        temp=reshape([data(index_43,215:216)'],1,[]);
        algo_pos_error_00  =double(typecast(uint8(temp),'int16')')/32768*10; 
        algo_pos_error_0  =[algo_pos_error_00; algo_pos_error_00];
        algo_pos_error_0  (1:2:end)=algo_pos_error_00;
        algo_pos_error_0  (2:2:end)=algo_pos_error_00; 
        
        temp=reshape([data(index_43,217:218)'],1,[]);
        algo_pos_error_10  =double(typecast(uint8(temp),'int16')')/32768*10; 
        algo_pos_error_1  =[algo_pos_error_10; algo_pos_error_10];
        algo_pos_error_1  (1:2:end)=algo_pos_error_10;
        algo_pos_error_1  (2:2:end)=algo_pos_error_10;  
        
        temp=reshape([data(index_43,219:220)'],1,[]);
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
        temp=reshape([data(index_20,165:166)'],1,[]);
        dAB_00=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_20,167:168)'],1,[]);
        dAB_01=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(index_20,169:170)'],1,[]);
        dAB_02=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_20,171:172)'],1,[]);
        dWB_00=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(index_20,173:174)'],1,[]);
        dWB_01=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(index_20,175:176)'],1,[]);
        dWB_02=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;         
        temp=reshape([data(index_21,153:154)'],1,[]);
        dAB_10=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_21,155:156)'],1,[]);
        dAB_11=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(index_21,157:158)'],1,[]);
        dAB_12=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_21,159:160)'],1,[]);
        dWB_10=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(index_21,161:162)'],1,[]);
        dWB_11=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(index_21,163:164)'],1,[]);
        dWB_12=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;         
         temp=reshape([data(index_21,165:166)'],1,[]);
        dAB_20=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_21,167:168)'],1,[]);
        dAB_21=double(typecast(uint8(temp),'int16')')/32768*2*-100;  
        temp=reshape([data(index_21,169:170)'],1,[]);
        dAB_22=double(typecast(uint8(temp),'int16')')/32768*2*100; 
        temp=reshape([data(index_21,171:172)'],1,[]);
        dWB_20=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;
        temp=reshape([data(index_21,173:174)'],1,[]);
        dWB_21=double(typecast(uint8(temp),'int16')')/32768*0.3*-5000;        
        temp=reshape([data(index_21,175:176)'],1,[]);
        dWB_22=double(typecast(uint8(temp),'int16')')/32768*0.3*5000;           
        temp=reshape([data(index_20,159:160)'],1,[]);
        gx_f=double(typecast(uint8(temp),'int16')')/32768*17.5*HD; 
        temp=reshape([data(index_20,161:162)'],1,[]);
        gy_f=double(typecast(uint8(temp),'int16')')/32768*-17.5*HD;  
        temp=reshape([data(index_20,163:164)'],1,[]);
        gz_f=double(typecast(uint8(temp),'int16')')/32768*-17.5*HD; 
        temp=reshape([data(index_21,145:146)'],1,[]);
        ax_f=double(typecast(uint8(temp),'int16')')/32768*80;%%80
        temp=reshape([data(index_21,147:148)'],1,[]);
        ay_f=double(typecast(uint8(temp),'int16')')/32768*-80;        
        temp=reshape([data(index_21,149:150)'],1,[]);
        az_f=double(typecast(uint8(temp),'int16')')/32768*-80; 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %MAg_YAW
        temp=reshape([data(index_40,209:210)'],1,[]);
        rtU_MagYaw_acc_0=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_40,211:212)'],1,[]);
        rtU_MagYaw_acc_1=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_40,213:214)'],1,[]);
        rtU_MagYaw_acc_2=double(typecast(uint8(temp),'int16')')/32768*80;
        temp=reshape([data(index_40,215:216)'],1,[]);
        rtU_MagYaw_mag_0=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,217:218)'],1,[]);
        rtU_MagYaw_mag_1=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,219:220)'],1,[]);
        rtU_MagYaw_mag_2=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_40,221:222)'],1,[]);
        rtU_MagYaw_out=double(typecast(uint8(temp),'int16')')/32768*4*HD;
 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %BMS
       temp = reshape([data(find(mod(Count,16)==0),289:289)'],1,[]);
ASOC=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.ASOC = ASOC; % create struct
temp = reshape([data(find(mod(Count,16)==0),290:290)'],1,[]);
RSOC=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
TempName.RSOC = RSOC; % create struct
temp = reshape([data(find(mod(Count,16)==0),291:292)'],1,[]);
SOH=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.SOH = SOH; % create struct
temp = reshape([data(find(mod(Count,16)==1),289:290)'],1,[]);
Average_Time_To_empty=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Average_Time_To_empty = Average_Time_To_empty; % create struct
temp = reshape([data(find(mod(Count,16)==1),291:292)'],1,[]);
Cell_Volt0=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt0 = Cell_Volt0; % create struct
temp = reshape([data(find(mod(Count,16)==2),289:290)'],1,[]);
Cell_Volt1=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt1 = Cell_Volt1; % create struct
temp = reshape([data(find(mod(Count,16)==2),291:292)'],1,[]);
Cell_Volt2=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt2 = Cell_Volt2; % create struct
temp = reshape([data(find(mod(Count,16)==3),289:290)'],1,[]);
Cell_Volt3=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt3 = Cell_Volt3; % create struct
temp = reshape([data(find(mod(Count,16)==3),291:292)'],1,[]);
Cell_Volt4=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt4 = Cell_Volt4; % create struct
temp = reshape([data(find(mod(Count,16)==4),289:290)'],1,[]);
Cell_Volt5=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cell_Volt5 = Cell_Volt5; % create struct
temp = reshape([data(find(mod(Count,16)==4),291:292)'],1,[]);
Cycle_Count=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Cycle_Count = Cycle_Count; % create struct
temp = reshape([data(find(mod(Count,16)==5),289:290)'],1,[]);
Design_Cap=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Design_Cap = Design_Cap; % create struct
temp = reshape([data(find(mod(Count,16)==5),291:292)'],1,[]);
Design_Volt=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Design_Volt = Design_Volt; % create struct
temp = reshape([data(find(mod(Count,16)==6),289:290)'],1,[]);
FCC=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.FCC = FCC; % create struct
temp = reshape([data(find(mod(Count,16)==6),291:292)'],1,[]);
FW_Version=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.FW_Version = FW_Version; % create struct
temp = reshape([data(find(mod(Count,16)==7),289:290)'],1,[]);
Manufacture_Date=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Manufacture_Date = Manufacture_Date; % create struct
temp = reshape([data(find(mod(Count,16)==7),291:292)'],1,[]);
Other_Status=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Other_Status = Other_Status; % create struct
temp = reshape([data(find(mod(Count,16)==8),289:290)'],1,[]);
Rem_Cap=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Rem_Cap = Rem_Cap; % create struct
temp = reshape([data(find(mod(Count,16)==8),291:292)'],1,[]);
Run_Time_TO_Empty=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Run_Time_TO_Empty = Run_Time_TO_Empty; % create struct
temp = reshape([data(find(mod(Count,16)==9),289:290)'],1,[]);
Safty_Status=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Safty_Status = Safty_Status; % create struct
temp = reshape([data(find(mod(Count,16)==9),291:292)'],1,[]);
Serial_Num=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Serial_Num = Serial_Num; % create struct
temp = reshape([data(find(mod(Count,2)==0),145:146)'],1,[]);
Temperature=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Temperature = Temperature; % create struct
temp = reshape([data(find(mod(Count,2)==0),147:148)'],1,[]);
Total_Cycles=double(typecast(uint8(temp),'uint16')')/1*1.0000000000;
TempName.Total_Cycles = Total_Cycles; % create struct
temp = reshape([data(find(mod(Count,2)==0),149:150)'],1,[]);
Voltage=double(typecast(uint8(temp),'uint16')')/1*1.0000000000/1000;
TempName.Voltage = Voltage; % create struct
temp = reshape([data(find(mod(Count,2)==0),151:154)'],1,[]);
Now_Current=double(typecast(uint8(temp),'int32')')/1*1.0000000000;
TempName.Now_Current = Now_Current; % create struct
temp = reshape([data(find(mod(Count,2)==0),155:158)'],1,[]);
Average_Current=double(typecast(uint8(temp),'int32')')/1*1.0000000000*(-0.001);
TempName.Average_Current = Average_Current*(-0.01); % create struct
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %airspeed
        temp=reshape([data(index_20,177:178)'],1,[]);
        diff_press_filtered_pa=double(typecast(uint8(temp),'int16')')/32768*1000;
        temp=reshape([data(index_20,179:180)'],1,[]);
        indicated_airspeed=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,181:182)'],1,[]);
        true_airspeed=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,183:184)'],1,[]);
        EAS_Algo=double(typecast(uint8(temp),'int16')')/32768*50;
        temp=reshape([data(index_20,185:186)'],1,[]);
        EAS2TAS_Algo=double(typecast(uint8(temp),'int16')')/32768*2;
        temp=reshape([data(index_20,189:190)'],1,[]);
        airspeed_used=double(typecast(uint8(temp),'int16')')/32768*100;
        temp=reshape([data(index_20,191:192)'],1,[]);
        diff_press_pa_raw=double(typecast(uint8(temp),'int16')')/32768*1000;
        temp=reshape([data(index_20,193:194)'],1,[]);
        diff_offset=double(typecast(uint8(temp),'int16')')/32768*300;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        temp=reshape([data(index_40,209:212)'],1,[]);
        algo_NAV_lon=double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(index_40,213:216)'],1,[]);
        algo_NAV_lat=double(typecast(uint8(temp),'int32')')*1e-7;
        temp=reshape([data(index_41,209:210)'],1,[]);
        algo_NAV_yaw=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(index_41,211:212)'],1,[]);
        algo_NAV_pitch=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(index_41,213:214)'],1,[]);
        algo_NAV_roll=double(typecast(uint8(temp),'int16')')/32768*200; 
        temp=reshape([data(index_41,219:220)'],1,[]);
        algo_NAV_alt=double(typecast(uint8(temp),'int16')')/32768*2000;
        temp=reshape([data(index_42,209:210)'],1,[]);
        algo_NAV_Vn=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(index_42,211:212)'],1,[]);
        algo_NAV_Ve=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(index_42,213:214)'],1,[]);
        algo_NAV_Vd=double(typecast(uint8(temp),'int16')')/32768*100;   
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %position 
        temp=reshape([data(index_82,197:198)'],1,[]);
        mission_item_seq=double(typecast(uint8(temp),'int16')');  
        temp=reshape([data(index_82,199:202)'],1,[]);
        mission_item_lat=double(typecast(uint8(temp),'single')');   
        temp=reshape([data(index_82,203:206)'],1,[]);
        mission_item_lon=double(typecast(uint8(temp),'single')'); 
        temp=reshape([data(index_83,197:198)'],1,[]);
        algo_loc_num=double(typecast(uint8(temp),'int16')');  
        temp=reshape([data(index_83,199:202)'],1,[]);
        algo_loc_lat=double(typecast(uint8(temp),'single')')/1e7;   
        temp=reshape([data(index_83,203:206)'],1,[]);
        algo_loc_lon=double(typecast(uint8(temp),'single')')/1e7;         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %out taskmode
        temp=reshape([data(index_160,229:230)'],1,[]);
        algo_currentPointNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_160,231:232)'],1,[]);
        algo_prePointNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_161,229:230)'],1,[]);
        algo_validPathNum=double(typecast(uint8(temp),'int16')'); 
        temp=reshape([data(index_161,231:232)'],1,[]);
        algo_headingCmd=double(typecast(uint8(temp),'int16')')/32768*6.3;         
        temp=reshape([data(index_162,229:230)'],1,[]);
        algo_distToGo=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_162,231:232)'],1,[]);
        algo_dz=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_163,229:230)'],1,[]);
        algo_groundspeedCmd=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_163,231:232)'],1,[]);
        algo_rollCmd=double(typecast(uint8(temp),'int16')')/32768*10000;           
        temp=reshape([data(index_164,229:230)'],1,[]);
        algo_turnRadiusCmd=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_164,231:232)'],1,[]);
        algo_heightCmd_sl=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_165,229:232)'],1,[]);
        algo_turnCenterLL_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(index_166,229:232)'],1,[]);
        algo_turnCenterLL_1 =double(typecast(uint8(temp),'int32')')*1e-7;                                    
        temp=reshape([data(index_167,229:230)'],1,[]);
        algo_dR_turn=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_167,231)'],1,[]);
        algo_uavMode=double(typecast(uint8(temp),'uint8')'); 
        temp=reshape([data(index_167,232)'],1,[]);
        algo_flightTaskMode_sl=double(typecast(uint8(temp),'uint8')'); 
        temp=reshape([data(index_168,229)'],1,[]);
        algo_flightControlMode =double(typecast(uint8(temp),'uint8')');    
        temp=reshape([data(index_168,230)'],1,[]);
        algo_AutoManualMode =double(typecast(uint8(temp),'uint8')');
        temp=reshape([data(index_168,231)'],1,[]);
        algo_comStatus =double(typecast(uint8(temp),'uint8')');
        temp=reshape([data(index_169,229:230)'],1,[]);
        algo_maxClimbSpeed_sl=double(typecast(uint8(temp),'int16')')/32768*100; 
        temp=reshape([data(index_1610,229:232)'],1,[]);
        algo_prePathPoint_LLA_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(index_1611,229:232)'],1,[]);
        algo_prePathPoint_LLA_1=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(index_1611,229:230)'],1,[]);
        algo_prePathPoint_LLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;  
        temp=reshape([data(index_1613,229:232)'],1,[]);
        algo_curPathPoint_LLA_0=double(typecast(uint8(temp),'int32')')*1e-7; 
        temp=reshape([data(index_1614,229:232)'],1,[]);
        algo_curPathPoint_LLA_1=double(typecast(uint8(temp),'int32')')*1e-7;      
        temp=reshape([data(index_1615,229:230)'],1,[]);
        algo_curPathPoint_LLA_2=double(typecast(uint8(temp),'int16')')/32768*10000; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %out task flight param
        temp=reshape([data(index_160,233:236)'],1,[]);
        algo_curHomeLLA_0=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(index_161,233:236)'],1,[]);
        algo_curHomeLLA_1=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(index_162,233:234)'],1,[]);
        algo_curHomeLLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;
        temp=reshape([data(index_162,235:236)'],1,[]);
        algo_curVelNED_0=double(typecast(uint8(temp),'int16')')/32768*50;         
        temp=reshape([data(index_163,233:234)'],1,[]);
        algo_curVelNED_1=double(typecast(uint8(temp),'int16')')/32768*50;  
        temp=reshape([data(index_163,235:236)'],1,[]);
        algo_curVelNED_2=double(typecast(uint8(temp),'int16')')/32768*30;         
        temp=reshape([data(index_164,233:234)'],1,[]);
        algo_curSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;  
        temp=reshape([data(index_164,235:236)'],1,[]);
        algo_curAirSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(index_165,233:234)'],1,[]);
        algo_curEuler_0=double(typecast(uint8(temp),'int16')')/32768*10;
        temp=reshape([data(index_165,235:236)'],1,[]);
        algo_curEuler_1=double(typecast(uint8(temp),'int16')')/32768*10;         
        temp=reshape([data(index_166,233:234)'],1,[]);
        algo_curEuler_2=double(typecast(uint8(temp),'int16')')/32768*10;       
        temp=reshape([data(index_166,235:236)'],1,[]);
        algo_curWB_0=double(typecast(uint8(temp),'int16')')/32768*10;         
        temp=reshape([data(index_167,233:234)'],1,[]);
        algo_curWB_1=double(typecast(uint8(temp),'int16')')/32768*10000;       
        temp=reshape([data(index_167,235:236)'],1,[]);
        algo_curWB_2=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(index_168,233:234)'],1,[]);
        algo_curPosNED_0=double(typecast(uint8(temp),'int16')')/32768*10000; 
        temp=reshape([data(index_168,235:236)'],1,[]);
        algo_curPosNED_1=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(index_169,233:234)'],1,[]);
        algo_curPosNED_2=double(typecast(uint8(temp),'int16')')/32768*10000;        
        temp=reshape([data(index_1610,233:236)'],1,[]);
        algo_curLLA_0=double(typecast(uint8(temp),'int32')')*1e-7;         
        temp=reshape([data(index_1611,233:236)'],1,[]);
        algo_curLLA_1=double(typecast(uint8(temp),'int32')')*1e-7;  
        temp=reshape([data(index_1612,233:234)'],1,[]);
        algo_curLLA_2=double(typecast(uint8(temp),'int16')')/32768*10000;         
        temp=reshape([data(index_1612,235:236)'],1,[]);
        algo_curGroundSpeed=double(typecast(uint8(temp),'int16')')/32768*10000;       
        temp=reshape([data(index_1613,233:234)'],1,[]);
        algo_curAccZ=double(typecast(uint8(temp),'int16')')/32768*10000;   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
temp = reshape([data(find(mod(Count,4)==0),239:242)'],1,[]);
Lon=double(typecast(uint8(temp),'int32')')/10000000*10000000.0000000000;
SL.OUT_FLIGHTPERF.Lon = Lon; % create struct
temp = reshape([data(find(mod(Count,4)==0),257:260)'],1,[]);
Lat=double(typecast(uint8(temp),'int32')')/10000000*10000000.0000000000;
SL.OUT_FLIGHTPERF.Lat = Lat; % create struct
temp = reshape([data(find(mod(Count,4)==0),261:264)'],1,[]);
height1=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.height = height1; % create struct
temp = reshape([data(find(mod(Count,4)==0),265:268)'],1,[]);
velN=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.velN = velN; % create struct
temp = reshape([data(find(mod(Count,4)==0),269:272)'],1,[]);
velE=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.velE = velE; % create struct
temp = reshape([data(find(mod(Count,4)==0),273:276)'],1,[]);
velD=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.velD = velD; % create struct
temp = reshape([data(find(mod(Count,4)==0),277:280)'],1,[]);
delta_lon=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.delta_lon = delta_lon; % create struct
temp = reshape([data(find(mod(Count,4)==0),281:284)'],1,[]);
delta_lat=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.delta_lat = delta_lat; % create struct
temp = reshape([data(find(mod(Count,4)==0),285:288)'],1,[]);
delta_height=double(typecast(uint8(temp),'single')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.delta_height = delta_height; % create struct
temp = reshape([data(find(mod(Count,4)==0),237:238)'],1,[]);
pDop=double(typecast(uint8(temp),'int16')')/32768*100.0000000000;
SL.OUT_FLIGHTPERF.pDop = pDop; % create struct
temp = reshape([data(find(mod(Count,4)==0),39:39)'],1,[]);
BESTPOS=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.BESTPOS = BESTPOS; % create struct
temp = reshape([data(find(mod(Count,4)==0),40:40)'],1,[]);
numSv=double(typecast(uint8(temp),'uint8')')/1*1.0000000000;
SL.OUT_FLIGHTPERF.numSv = numSv; % create struct
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
temp = reshape([data(find(mod(Count,4)==3),121:124)'],1,[]);
task_1ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
SL.SystemInfo.task_1ms_total_cnt = task_1ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),125:128)'],1,[]);
task_4ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
SL.SystemInfo.task_4ms_total_cnt = task_4ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),129:132)'],1,[]);
task_12ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
SL.SystemInfo.task_12ms_total_cnt = task_12ms_total_cnt; % create struct
temp = reshape([data(find(mod(Count,4)==3),133:136)'],1,[]);
task_100ms_total_cnt=double(typecast(uint8(temp),'uint32')')/1*1.0000000000;
SL.SystemInfo.task_100ms_total_cnt = task_100ms_total_cnt; % create struct
% len=length(task_1ms_total_cnt);
% hold on
% plot(diff(task_1ms_total_cnt))
% % plot(diff(task_4ms_total_cnt))
% plot(diff(task_12ms_total_cnt))
% plot((task_1ms_total_cnt),'*')
% plot((task_4ms_total_cnt),'*')
% plot((task_12ms_total_cnt),'*')
% plot(task_4ms_total_cnt./len)
%  hold on
%  plot(task_1ms_total_cnt./task_4ms_total_cnt)
%  plot(task_1ms_total_cnt./task_12ms_total_cnt)
%  plot(task_1ms_total_cnt./task_100ms_total_cnt)
%  plot(diff(save_time))


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
        temp=reshape([data(index_80,197:198)'],1,[]);
        imu_tmp=double(typecast(uint8(temp),'int16')')/32768*100;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        time_imu=save_time;
        time_baro=save_time(1:2:end);
        time_mag=save_time(1:4:end);
        time_radar=save_time(1:4:end);
        time_ublox=save_time(1:4:end);
        time_algo=save_time(1:2:end);
        time_bms1=save_time(1:16:end);
        time_bms2=save_time(1:2:end);
        time_arspeed=save_time(1:2:end);
        time_mag_yaw=save_time(1:4:end);
        time_sl_att=save_time(1:4:end);
        time_sl_DataFusion=save_time(1:end);
        time_imu_tmp=save_time(1:8:end);
        time_pos_tmp=save_time(1:8:end);
        time_sl_taskmode=save_time(1:16:end);
        time_sl_task_flight=save_time(1:16:end);
        time_um482=save_time(1:4:end);
               
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
    
    
    mag1_abs=hypot(hypot(mag1_x ,mag1_y), mag1_z);
    mag2_abs=hypot(hypot(mag2_x ,mag2_y), mag2_z);
    mag1x_abs=hypot(hypot(mag1_xx, mag1_yx), mag1_zx);
    mag2x_abs=hypot(hypot(mag2_xx, mag2_yx), mag2_zx);
   
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
        'mag1_abs';
        'mag2_abs';
        'mag1x_abs';
        'mag2x_abs';       
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
   data_bms_txt1={
        'time_bms1'      
        'ASOC';
        'RSOC';
        'SOH';
        'Average_Time_To_empty';
        'Cell_Volt0';
        'Cell_Volt1';
        'Cell_Volt2';
        'Cell_Volt3';
        'Cell_Volt4';
        'Cell_Volt5';
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
        };
   
    data_bms_txt2={
        'time_bms2'
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
        'src_tilt';
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
%         'algo_PathMode';
        'algo_flightTaskMode';
        'GSConnected';
        'Remote_Disconnect_Status';
        'remote_time_cnt';
        'algo_vel_forward_last_pct';
%         'FC_VERSION';
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
       'algo_plane_mode';
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
    data_um482={
    'time_um482';
    'SL.OUT_FLIGHTPERF.Lon'; 
    'SL.OUT_FLIGHTPERF.Lat';
    'SL.OUT_FLIGHTPERF.height'; 
    'SL.OUT_FLIGHTPERF.velN';
    'SL.OUT_FLIGHTPERF.velE';
    'SL.OUT_FLIGHTPERF.velD';
    'SL.OUT_FLIGHTPERF.delta_lon'; 
    'SL.OUT_FLIGHTPERF.delta_lat'; 
    'SL.OUT_FLIGHTPERF.delta_height';
    'SL.OUT_FLIGHTPERF.pDop'; 
    'SL.OUT_FLIGHTPERF.BESTPOS'; 
    'SL.OUT_FLIGHTPERF.numSv';
};


  % --------------------------------------------------algo-------------------
data = [];
   for i=1:length(data_ck_txt)
       data(:,i)=eval(data_ck_txt{i,1}); 
   end
   head=[data_ck_txt{1}];
   for i=2:length(data_ck_txt)
       head=[head ,' ',data_ck_txt{i}];
   end 
   head=[head ,'\n'];
data_ck=data(1:2:end,1:i);
fid=fopen([filename,'V1000.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000.dat'],'data_ck','-ascii','-append' )

% --------------------------------------------------imu txt----------------
data_1 = [];
for i=1:length(data_imu_txt)
        data_1(:,i)=eval(data_imu_txt{i,1}); 
        end
        head=[data_imu_txt{1}];
        for i=2:length(data_imu_txt)
         head=[head ,' ',data_imu_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_1(:,1:i);
fid=fopen([filename,'V1000_IMU.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_IMU.dat'],'data_ck','-ascii','-append' )
   
% --------------------------------------------------ublox txt--------------
data_2 = [];
for i=1:length(data_ublox_txt)
        data_2(:,i)=eval(data_ublox_txt{i,1}); 
        end
        head=[data_ublox_txt{1}];
        for i=2:length(data_ublox_txt)
         head=[head ,' ',data_ublox_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_2(:,1:i);
fid=fopen([filename,'V1000_UBLOX.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_UBLOX.dat'],'data_ck','-ascii','-append' )

% --------------------------------------------------mag txt----------------
data_3 = [];
for i=1:length(data_mag_txt)
        data_3(:,i)=eval(data_mag_txt{i,1}); 
        end
        head=[data_mag_txt{1}];
        for i=2:length(data_mag_txt)
         head=[head ,' ',data_mag_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_3(:,1:i);
fid=fopen([filename,'V1000_MAG.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_MAG.dat'],'data_ck','-ascii','-append' )
% --------------------------------------------------radar txt--------------
data_4 = [];
for i=1:length(data_radar_txt)
        data_4(:,i)=eval(data_radar_txt{i,1}); 
        end
        head=[data_radar_txt{1}];
        for i=2:length(data_radar_txt)
         head=[head ,' ',data_radar_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_4(:,1:i);
fid=fopen([filename,'V1000_RADAR.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_RADAR.dat'],'data_ck','-ascii','-append' )

% --------------------------------------------------baror txt--------------
data_5 = [];
for i=1:length(data_baro_txt)
        data_5(:,i)=eval(data_baro_txt{i,1}); 
        end
        head=[data_baro_txt{1}];
        for i=2:length(data_baro_txt)
         head=[head ,' ',data_baro_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_5(:,1:i);
fid=fopen([filename,'V1000_BARO.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_BARO.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_6 = [];
for i=1:length(data_bms_txt1)
        data_6(:,i)=eval(data_bms_txt1{i,1}); 
        end
        head=[data_bms_txt1{1}];
        for i=2:length(data_bms_txt1)
         head=[head ,' ',data_bms_txt1{i}];
        end 
        head=[head ,'\n'];
data_ck=data_6(:,1:i);
fid=fopen([filename,'V1000_bms.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_bms.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_7 = [];
for i=1:length(data_arspeed_txt)
        data_7(:,i)=eval(data_arspeed_txt{i,1}); 
        end
        head=[data_arspeed_txt{1}];
        for i=2:length(data_arspeed_txt)
         head=[head ,' ',data_arspeed_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_7(:,1:i);
fid=fopen([filename,'V1000_arspeed.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_arspeed.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_8 = [];
% for i=1:length(data_mag_yaw_txt)
%         data_8(:,i)=eval(data_mag_yaw_txt{i,1}); 
%         end
%         head=[data_mag_yaw_txt{1}];
%         for i=2:length(data_mag_yaw_txt)
%          head=[head ,' ',data_mag_yaw_txt{i}];
%         end 
%         head=[head ,'\n'];
% data_ck=data_8(:,1:i);
% fid=fopen([filename,'V1000_mag_yaw.dat'],'w');
% fprintf(fid,head);
% %     [count,num]=size(data_ck);
% %     for i=1:count
% %         for j=1:num
% %             fprintf(fid,'%f ',data_ck(i,j));
% %         end
% %         fprintf(fid,'\n');
% %     end
%     fclose(fid); 
% save([filename,'V1000_mag_yaw.dat'],'data_ck','-ascii','-append' )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_9 = [];
for i=1:length(data_sl_att_txt)
        data_9(:,i)=eval(data_sl_att_txt{i,1}); 
        end
        head=[data_sl_att_txt{1}];
        for i=2:length(data_sl_att_txt)
         head=[head ,' ',data_sl_att_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_9(:,1:i);
fid=fopen([filename,'V1000_sl_att_txt.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_sl_att_txt.dat'],'data_ck','-ascii','-double','-append' )
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_10 = [];
% for i=1:length(data_sl_DataFusion_txt)
%         data_10(:,i)=eval(data_sl_DataFusion_txt{i,1});        
% end
%         head=[data_sl_DataFusion_txt{1}];
%         for i=2:length(data_sl_DataFusion_txt)
%          head=[head ,' ',data_sl_DataFusion_txt{i}];
%         end 
%         head=[head ,'\n'];
% data_ck=data_10(:,1:i);
% fid=fopen([filename,'V1000_sl_DataFusion_txt.dat'],'w');
% fprintf(fid,head);
% %     [count,num]=size(data_ck);
% %     for i=1:count
% %         for j=1:num
% %             fprintf(fid,'%f ',data_ck(i,j));
% %         end
% %         fprintf(fid,'\n');
% %     end
%     fclose(fid); 
% save([filename,'V1000_sl_DataFusion_txt.dat'],'data_ck','-ascii','-append' )
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_11 = [];
% for i=1:length(data_imu_tmp_txt)
%         data_11(:,i)=eval(data_imu_tmp_txt{i,1});        
% end
%         head=[data_imu_tmp_txt{1}];
%         for i=2:length(data_imu_tmp_txt)
%          head=[head ,' ',data_imu_tmp_txt{i}];
%         end 
%         head=[head ,'\n'];
% data_ck=data_11(:,1:i);
% fid=fopen([filename,'V1000_imu_tmp_txt.dat'],'w');
% fprintf(fid,head);
% %     [count,num]=size(data_ck);
% %     for i=1:count
% %         for j=1:num
% %             fprintf(fid,'%f ',data_ck(i,j));
% %         end
% %         fprintf(fid,'\n');
% %     end
%     fclose(fid); 
% save([filename,'V1000_imu_tmp_txt.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_12 = [];
for i=1:length(data_position_txt)
        data_12(:,i)=eval(data_position_txt{i,1});        
end
        head=[data_position_txt{1}];
        for i=2:length(data_position_txt)
         head=[head ,' ',data_position_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_12(:,1:i);
fid=fopen([filename,'V1000_position_txt.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_position_txt.dat'],'data_ck','-ascii','-append' )


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data_13 = [];
% for i=1:length(data_sl_temp)
%         data_13(:,i)=eval(data_sl_temp{i,1});        
% end
%         head=[data_sl_temp{1}];
%         for i=2:length(data_sl_temp)
%          head=[head ,' ',data_sl_temp{i}];
%         end 
%         head=[head ,'\n'];
% data_ck=data_13(:,1:i);
% fid=fopen([filename,'V1000_sl_temp.dat'],'w');
% fprintf(fid,head);
% %     [count,num]=size(data_ck);
% %     for i=1:count
% %         for j=1:num
% %             fprintf(fid,'%f ',data_ck(i,j));
% %         end
% %         fprintf(fid,'\n');
% %     end
%     fclose(fid); 
% save([filename,'V1000_sl_temp.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_14 = [];
for i=1:length(data_sl_taskmode)
        data_14(:,i)=eval(data_sl_taskmode{i,1});        
end
        head=[data_sl_taskmode{1}];
        for i=2:length(data_sl_taskmode)
         head=[head ,' ',data_sl_taskmode{i}];
        end 
        head=[head ,'\n'];
data_ck=data_14(:,1:i);
fid=fopen([filename,'V1000_sl_taskmode.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_sl_taskmode.dat'],'data_ck','-ascii','-append' )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_15 = [];
for i=1:length(data_sl_task_flight)
        data_15(:,i)=eval(data_sl_task_flight{i,1});        
end
        head=[data_sl_task_flight{1}];
        for i=2:length(data_sl_task_flight)
         head=[head ,' ',data_sl_task_flight{i}];
        end 
        head=[head ,'\n'];
data_ck=data_15(:,1:i);
fid=fopen([filename,'V1000_sl_task_flight.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([filename,'V1000_sl_task_flight.dat'],'data_ck','-ascii','-append' )
end