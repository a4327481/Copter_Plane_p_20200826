%  
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mat 格式转换为画图
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
C = fread(fileID);
fclose(fileID);
len=length(C);
k=1;
UTC.mweek=zeros(len,1);
UTC.weekn=zeros(len,1);
UTC.ns=zeros(len,1);
UTC.len=zeros(len,1);
UTC.Position_Flags_1=zeros(len,1);
UTC.Position_Flags_2=zeros(len,1);
UTC.Initialized_number=zeros(len,1);
UTC.LatLng_head=zeros(len,1);
UTC.Lat=zeros(len,1);
UTC.Lng=zeros(len,1);
UTC.Alt=zeros(len,1);
i=1;
while(i<len)
%     if (C(i)==2)&&(C(i+1)==40)&&(C(i+2)==64)
%         lenj=C(i+3)-3;
%         for j=1:lenj
%             if((C(i+6+j)==16)&&(C(i+6+j+1)==09))
%             UTC.mweek(k)=C(i+6+j+2)*2^24+C(i+6+j+3)*2^16+C(i+6+j+4)*2^8+C(i+6+j+5);
%             UTC.mweek(k)=UTC.mweek(k);
%             UTC.weekn(k)=C(i+6+j+6)*2^8+C(i+6+j+7);
%             k=k+1;            
%             end       
%         end
%     end    if (C(i)==2)&&(C(i+1)==40)&&(C(i+2)==64)
      
        if (C(i)==2) &&(C(i+2)==64)
            lenj=C(i+3)-3;
            for j=1
                if((C(i+6+j)==1)&&(C(i+6+j+1)==10))
                    UTC.mweek(k)=C(i+6+j+2)*2^24+C(i+6+j+3)*2^16+C(i+6+j+4)*2^8+C(i+6+j+5);
                    UTC.mweek(k)=UTC.mweek(k);
                    UTC.weekn(k)=C(i+6+j+6)*2^8+C(i+6+j+7);
                    UTC.ns(k)=C(i+6+j+8);
                    UTC.Position_Flags_1(k)=C(i+6+j+9);
                    UTC.Position_Flags_2(k)=C(i+6+j+10);
                    UTC.Initialized_number(k)=C(i+6+j+11);
                    UTC.len(k)=lenj;
                    UTC.LatLng_head(k)=C(i+6+j+12)+C(i+6+j+13);
                    UTC.Lat(k)=typecast(fliplr(uint8(C(i+6+j+14:i+6+j+21))'),'double')*180/pi;
                    UTC.Lng(k)=typecast(fliplr(uint8(C(i+6+j+22:i+6+j+29))'),'double')*180/pi;
                    UTC.Alt(k)=typecast(fliplr(uint8(C(i+6+j+30:i+6+j+37))'),'double');
                    
                    k=k+1;
                    
                end
                
            end          
        end
    i=i+1;       
end
 UTC.mweek(k:end)=[];
 UTC.weekn(k:end)=[];
 UTC.len(k:end)=[];
 UTC.ns(k:end)=[];
 UTC.Position_Flags_1(k:end)=[];
UTC.Position_Flags_2(k:end)=[];
UTC.Initialized_number(k:end)=[];
UTC.LatLng_head(k:end)=[];
UTC.Lat(k:end)=[];
UTC.Lng(k:end)=[];
UTC.Alt(k:end)=[];


%  plot(diff(UTC.mweek),'o')
data_ck=[(1:k-1)',UTC.weekn,UTC.mweek UTC.ns UTC.Position_Flags_1 UTC.Position_Flags_2 UTC.Initialized_number UTC.LatLng_head UTC.Lat UTC.Lng UTC.Alt];
fid=fopen([PathName,'\\',FileName,'UTC'],'w');
fprintf(fid,'index weeek mweek ns Position_Flags_1 Position_Flags_2 Initialized_number LatLng_head Lat Lng Alt\n');
    [count,~]=size(data_ck);
     for i=1:count
        fprintf(fid,' %d %d %d %d %d %d %d %d %.9f %.9f %.9f\n',data_ck(i,:));
    end
fclose(fid);
% save([PathName,'\\',FileName,'UTC'],'data_ck','-ascii','-append' );
