function servo_read()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

fileID = fopen([PathName,'\\',FileName]);
C = textscan(fileID,'%s');
fclose(fileID);

BLOCK_SIZE=5;
len=length(C{1,1})-BLOCK_SIZE;
m=floor(len/BLOCK_SIZE);
t1=zeros(m,1);
state1=zeros(m,1);
ecs_rpm1=zeros(m,1);
ecs_imv1=zeros(m,1);
ecs_ppm1=zeros(m,1);
t2=zeros(m,1);
state2=zeros(m,1);
ecs_rpm2=zeros(m,1);
ecs_imv2=zeros(m,1);
ecs_ppm2=zeros(m,1);
t3=zeros(m,1);
state3=zeros(m,1);
ecs_rpm3=zeros(m,1);
ecs_imv3=zeros(m,1);
ecs_ppm3=zeros(m,1);
t4=zeros(m,1);
state4=zeros(m,1);
ecs_rpm4=zeros(m,1);
ecs_imv4=zeros(m,1);
ecs_ppm4=zeros(m,1);
t5=zeros(m,1);
state5=zeros(m,1);
ecs_rpm5=zeros(m,1);
ecs_imv5=zeros(m,1);
ecs_ppm5=zeros(m,1);
i=1;
j1=1;
j2=1;
j3=1;
j4=1;
j5=1;
header.a='x07,x|';
header.ESC1='0040';
header.ESC2='0080';
header.ESC3='00C0';
header.ESC4='0100';
header.ESC5='0180';
while(i<len)
    lenj=length(C{1,1}{i});
    if(lenj>20)

        if (strcmp(C{1,1}{i}(end-5:end),header.a))
            
            switch   C{1,1}{i}(end-19:end-16)
                case header.ESC1
                    t1(j1,1)=datenum(C{1,1}{i}(14:25),'HH:MM:SS.FFF');
                    temp=sscanf(C{1,1}{i+1},'%x');
                    state1(j1,1)=temp;                    
                    tempa=sscanf(C{1,1}{i+2},'%x');
                    tempb=sscanf(C{1,1}{i+3},'%x');
                    ecs_rpm1(j1,1)=typecast(uint8([tempa,tempb]),'uint16');
                    
                    tempa=sscanf(C{1,1}{i+4},'%x');
                    tempb=sscanf(C{1,1}{i+5},'%x');
                    ecs_imv1(j1,1)=double(typecast(uint8([tempa,tempb]),'uint16'))/1000;
                    
                    tempa=sscanf(C{1,1}{i+6},'%x');
                    tempb=sscanf(C{1,1}{i+7},'%x');
                    ecs_ppm1(j1,1)=typecast(uint8([tempa,tempb]),'uint16');
                    j1=j1+1;
                    i=i+8;       
                case header.ESC2 
                    t2(j2,1)=datenum(C{1,1}{i}(14:25),'HH:MM:SS.FFF');
                    temp=sscanf(C{1,1}{i+1},'%x');
                    state2(j2,1)=temp;
                    
                    tempa=sscanf(C{1,1}{i+2},'%x');
                    tempb=sscanf(C{1,1}{i+3},'%x');
                    ecs_rpm2(j2,1)=typecast(uint8([tempa,tempb]),'uint16');
                    
                    tempa=sscanf(C{1,1}{i+4},'%x');
                    tempb=sscanf(C{1,1}{i+5},'%x');
                    ecs_imv2(j2,1)=double(typecast(uint8([tempa,tempb]),'uint16'))/1000;
                    
                    tempa=sscanf(C{1,1}{i+6},'%x');
                    tempb=sscanf(C{1,1}{i+7},'%x');
                    ecs_ppm2(j2,1)=typecast(uint8([tempa,tempb]),'uint16');
                    j2=j2+1;
                    i=i+8;
                case header.ESC3
                    t3(j3,1)=datenum(C{1,1}{i}(14:25),'HH:MM:SS.FFF');
                    temp=sscanf(C{1,1}{i+1},'%x');
                    state3(j3,1)=temp;
                    
                    tempa=sscanf(C{1,1}{i+2},'%x');
                    tempb=sscanf(C{1,1}{i+3},'%x');
                    ecs_rpm3(j3,1)=typecast(uint8([tempa,tempb]),'uint16');
                    
                    tempa=sscanf(C{1,1}{i+4},'%x');
                    tempb=sscanf(C{1,1}{i+5},'%x');
                    ecs_imv3(j3,1)=double(typecast(uint8([tempa,tempb]),'uint16'))/1000;
                    
                    tempa=sscanf(C{1,1}{i+6},'%x');
                    tempb=sscanf(C{1,1}{i+7},'%x');
                    ecs_ppm3(j3,1)=typecast(uint8([tempa,tempb]),'uint16');
                    j3=j3+1;
                    i=i+8;  
                    
                case header.ESC4
                    t4(j4,1)=datenum(C{1,1}{i}(14:25),'HH:MM:SS.FFF');
                    temp=sscanf(C{1,1}{i+1},'%x');
                    state4(j4,1)=temp;
                    
                    tempa=sscanf(C{1,1}{i+2},'%x');
                    tempb=sscanf(C{1,1}{i+3},'%x');
                    ecs_rpm4(j4,1)=typecast(uint8([tempa,tempb]),'uint16');
                    
                    tempa=sscanf(C{1,1}{i+4},'%x');
                    tempb=sscanf(C{1,1}{i+5},'%x');
                    ecs_imv4(j4,1)=double(typecast(uint8([tempa,tempb]),'uint16'))/1000;
                    
                    tempa=sscanf(C{1,1}{i+6},'%x');
                    tempb=sscanf(C{1,1}{i+7},'%x');
                    ecs_ppm4(j4,1)=typecast(uint8([tempa,tempb]),'uint16');
                    j4=j4+1;
                    i=i+8;                      
                case header.ESC5
                    t5(j5,1)=datenum(C{1,1}{i}(14:25),'HH:MM:SS.FFF');
                    temp=sscanf(C{1,1}{i+1},'%x');
                    state5(j5,1)=temp;
                    
                    tempa=sscanf(C{1,1}{i+2},'%x');
                    tempb=sscanf(C{1,1}{i+3},'%x');
                    ecs_rpm5(j5,1)=typecast(uint8([tempa,tempb]),'uint16');
                    
                    tempa=sscanf(C{1,1}{i+4},'%x');
                    tempb=sscanf(C{1,1}{i+5},'%x');
                    ecs_imv5(j5,1)=double(typecast(uint8([tempa,tempb]),'uint16'))/1000;
                    
                    tempa=sscanf(C{1,1}{i+6},'%x');
                    tempb=sscanf(C{1,1}{i+7},'%x');
                    ecs_ppm5(j5,1)=typecast(uint8([tempa,tempb]),'uint16');
                    j5=j5+1;
                    i=i+8;                     
                otherwise               
            end

        end
    end
    i=i+1;       
end
t1(j1:end)=[];
state1(j1:end)=[];
ecs_rpm1(j1:end)=[];
ecs_imv1(j1:end)=[];
ecs_ppm1(j1:end)=[];
t2(j2:end)=[];
state2(j2:end)=[];
ecs_rpm2(j2:end)=[];
ecs_imv2(j2:end)=[];
ecs_ppm2(j2:end)=[];
t3(j3:end)=[];
state3(j3:end)=[];
ecs_rpm3(j3:end)=[];
ecs_imv3(j3:end)=[];
ecs_ppm3(j3:end)=[];
t4(j4:end)=[];
state4(j4:end)=[];
ecs_rpm4(j4:end)=[];
ecs_imv4(j4:end)=[];
ecs_ppm4(j4:end)=[];
t5(j5:end)=[];
state5(j5:end)=[];
ecs_rpm5(j5:end)=[];
ecs_imv5(j5:end)=[];
ecs_ppm5(j5:end)=[];
a=487+3-0.7+1;
% t1=1:j1-1*0.05;
if (~isempty(t1))
data_ck=[(t1-t1(1))*1e5+a state1 ecs_rpm1 ecs_imv1 ecs_ppm1];
fid=fopen([PathName,'\\',FileName(1:end-4),'_servo1.dat'],'w');
fprintf(fid,'t state ecs_rpm ecs_imv ecs_ppm\n');
fclose(fid);
save([PathName,'\\',FileName(1:end-4),'_servo1.dat'],'data_ck','-ascii','-append' )
end
% t2=1:j2-1*0.05;
if (~isempty(t2))
data_ck=[(t2-t2(1))*1e5+a state2 ecs_rpm2 ecs_imv2 ecs_ppm2];
fid=fopen([PathName,'\\',FileName(1:end-4),'_servo2.dat'],'w');
fprintf(fid,'t state ecs_rpm ecs_imv ecs_ppm\n');
fclose(fid);
save([PathName,'\\',FileName(1:end-4),'_servo2.dat'],'data_ck','-ascii','-append' )
end

% t3=1:j3-1*0.05;
if (~isempty(t3))
data_ck=[(t3-t3(1))*1e5+a state3 ecs_rpm3 ecs_imv3 ecs_ppm3];
fid=fopen([PathName,'\\',FileName(1:end-4),'_servo3.dat'],'w');
fprintf(fid,'t state ecs_rpm ecs_imv ecs_ppm\n');
fclose(fid);
save([PathName,'\\',FileName(1:end-4),'_servo3.dat'],'data_ck','-ascii','-append' )
end
% t4=1:j4-1*0.05;
if (~isempty(t4))
data_ck=[(t4-t4(1))*1e5+a state4 ecs_rpm4 ecs_imv4 ecs_ppm4];
fid=fopen([PathName,'\\',FileName(1:end-4),'_servo4.dat'],'w');
fprintf(fid,'t state ecs_rpm ecs_imv ecs_ppm\n');
fclose(fid);
save([PathName,'\\',FileName(1:end-4),'_servo4.dat'],'data_ck','-ascii','-append' )
end
% t5=1:j5-1*0.05;
if (~isempty(t5))
data_ck=[(t5-t5(1))*1e5+a state5 ecs_rpm5 ecs_imv5 ecs_ppm5];
fid=fopen([PathName,'\\',FileName(1:end-4),'_servo5.dat'],'w');
fprintf(fid,'t state ecs_rpm ecs_imv ecs_ppm\n');
fclose(fid);
save([PathName,'\\',FileName(1:end-4),'_servo5.dat'],'data_ck','-ascii','-append' )
end
end