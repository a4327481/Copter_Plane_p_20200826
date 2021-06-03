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
parserData = fieldnames(V10Log);
for i = 1:length(parserData)
    V10Log_parserData=V10Log.(parserData{i});
    V10Log_parserData_i=fieldnames(V10Log_parserData);
    V10Log_parserData=alignDimension_max(V10Log_parserData);
    
    data = V10Log_parserData.(V10Log_parserData_i{1,1});
    for j=2:length(V10Log_parserData_i)
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
    data_ck(:,1)=data_ck(:,1)/1e4;
    fid=fopen([out_filePath(1:end-4),'_',parserData{i},'.dat'],'w');
    fprintf(fid,head);
    fclose(fid);
    save([out_filePath(1:end-4),'_',parserData{i},'.dat'],'data_ck','-ascii','-append' )

% 	fprintf('output:		%s\n',parserData{i});
% 	assignin('base',parserData{i},V10Log.(parserData{i}));
end

% plot(ALD1(1:end-1,3)/1e4,diff(ALD1(:,3))/10)
