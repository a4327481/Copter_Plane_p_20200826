%V1000 to txt  
% Display surf plot of the currently selected data.
%mat 格式转换为画图
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
 out_filePath=data_read_V10();
V10Log = V10_decode_auto(out_filePath);
 time_algo=V10Log.BUS_CTRL.TimeUS*1e-6;
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    V10Log_parserData=V10Log.(parserData{i});
    V10Log_parserData_i=fieldnames(V10Log_parserData);
    V10Log_parserData=alignDimension(V10Log_parserData);
    data = [];
    for j=1:length(V10Log_parserData_i)
        temp_data=V10Log_parserData.(V10Log_parserData_i{j,1});
        data=[ data temp_data];
    end
    
    head=[V10Log_parserData_i{1}];
    for j=2:length(V10Log_parserData_i)
        len=size(V10Log_parserData.(V10Log_parserData_i{j,1}),2);
        for jj=1:len
        head=[head ,' ',V10Log_parserData_i{j}];
        end
    end
    head=[head ,'\n'];
    data_ck=data(1:1:end,:);
    data_ck(:,1)=data_ck(:,1)/1e6;
    fid=fopen([out_filePath(1:end-4),'_',parserData{i},'.dat'],'w');
    fprintf(fid,head);
    fclose(fid);
    save([out_filePath(1:end-4),'_',parserData{i},'.dat'],'data_ck','-ascii','-append' )

% 	fprintf('output:		%s\n',parserData{i});
% 	assignin('base',parserData{i},V10Log.(parserData{i}));
end


