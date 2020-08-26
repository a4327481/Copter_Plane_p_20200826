% global PathName
% if PathName~=0
%     cd(PathName);
%     [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 读取雷达数据
% else
%     [FileName,PathName,~] = uigetfile('*.*'); % 读取雷达数据
% end
% if FileName==0
%     return;
% end
% load([PathName,'\\',FileName]);


data_max_txt={
'out.tout';
'out.MAG_YAW';
};
data_7 = [];
for i=1:length(data_max_txt)
        data_7(:,i)=eval(data_max_txt{i,1}); 
        end
        head=[data_max_txt{1}];
        for i=2:length(data_max_txt)
         head=[head ,' ',data_max_txt{i}];
        end 
        head=[head ,'\n'];
data_ck=data_7(:,1:i);
fid=fopen([PathName,'\\',FileName,'V1000_data_max_txt.dat'],'w');
fprintf(fid,head);
%     [count,num]=size(data_ck);
%     for i=1:count
%         for j=1:num
%             fprintf(fid,'%f ',data_ck(i,j));
%         end
%         fprintf(fid,'\n');
%     end
    fclose(fid); 
save([PathName,'\\',FileName,'V1000_data_max_txt.dat'],'data_ck','-ascii','-append' )