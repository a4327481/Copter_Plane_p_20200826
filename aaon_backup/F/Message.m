 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% global PathName
% if PathName~=0
%     cd(PathName);
%     [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % ��ȡ�״�����
% else
%     [FileName,PathName,~] = uigetfile('*.*'); % ��ȡ�״�����
% end
% if FileName==0
%     return;
% end
% 
% fileID = fopen([PathName,'\\',FileName]);
% C = fread(fileID);
% fclose(fileID);

modelname = 'read_CMD';
dllname = ['\codegen\dll\',modelname,'\',modelname];
dllheader = [dllname,'.h'];
loadlibrary(dllname,dllheader)
% libfunctions read_CMD -full
% calllib(modelname,modelname,C);
[cmd,k]=read_CMD(C);