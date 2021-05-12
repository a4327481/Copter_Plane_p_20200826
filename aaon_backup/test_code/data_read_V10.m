%data_read_V10
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
filePath=[PathName,'\\',FileName];
%%%%%%%%%%%%%生成协议%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N=1e8;
% u=0;
% temp_N=N;
% fileID = fopen(filePath);
% while (temp_N==N)
%     [C ,temp_N]= fread(fileID,N,'uint8');
%     u=u+1;
% end
%     fclose(fileID);
% L=(u-1)*N+temp_N;

HEAD1='A3';
HEAD2='95';
MsgType ='80';
N=1e6;%一次读取的数据长度
fileID = fopen(filePath);
[C ,N]= fread(fileID,N,'uint8');
C=uint8(C);
i=1;
j=1;
k=1;
while(i<=N)
    if((C(i)==hex2dec(HEAD1))&&(C(i+1)==hex2dec(HEAD2))&&(C(i+2)==hex2dec(MsgType))&&(C(i+3)>hex2dec('80')))
        MsgID(j,1)=C(i+3);
        MsgLen(j,1)=double(C(i+4));
        temp_Format=C(i+9:i+24)';
        temp_Lables=C(i+25:i+88)';
        temp_Name  =char(C(i+5:i+8))';
        
        temp_Name1=temp_Name(temp_Name~=0);
        Name(j,1)=string(char(temp_Name1));
        temp_Format1=temp_Format(temp_Format~=0);
        Format_len(j,1)=length(temp_Format1);
        Format(j,1)=string(char(temp_Format1));
        for il=1:Format_len(j,1)
            [out ,outp,lenj] = Format_to_type(Format{j,1}(il));
            Format_type(j,il)=string(out);
            Format_leni(j,il)=lenj;
        end 
        temp_Lables1=temp_Lables(temp_Lables~=0);
        Lables{j,1}=char(temp_Lables1); 
        temp_label=string(char(temp_Lables1));
        temp_label=(strsplit(temp_label,','))';
        temp_label=cellstr(temp_label);
        assignin('base',[char(temp_Name1),'_label'],temp_label)
        assignin('base',[char(temp_Name1)],zeros(2,Format_len(j,1)))
        assignin('base',[char(temp_Name1),'i'],1)
        Name_len(j)=0;
        Name_len_k(j)=1;
        j=j+1;
        i=i+89;
        k=i;
    else
        i=i+1;
    end
end
temp_C=C(k:end);
%%%%%%%%%%%%%%按照协议解析log%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=1;
i=1;
temp_N=N;
Nlen=20000;
u=1;
while (temp_N==N)
    [C ,temp_N]= fread(fileID,N,'uint8');
    C=uint8(C);
    temp_C = [temp_C;C];
    C=[];
    len=length(temp_C);
    i=1;
    while(i<=len-max(MsgLen))
        if((temp_C(i)==163)&&(temp_C(i+1)==149))
            temp_MsgID=temp_C(i+2);
            tempMsgID=find(MsgID==temp_MsgID,1);
            if(tempMsgID)            
                Name_len(tempMsgID)=Name_len(tempMsgID)+1;
%                 temp_Name=Name(tempMsgID);
%                 temp_Format=Format(tempMsgID);
%                 temp_Format_len=Format_len(tempMsgID);
                temp_MsgLen=MsgLen(tempMsgID);
%                 temp_Format_type=Format_type((tempMsgID),:);
%                 temp_Format_leni=Format_leni((tempMsgID),:);
%                 tmep_dataC=temp_C(i+3:i+temp_MsgLen-1)';
%                 temp_data=zeros(1,temp_Format_len);
 
%                 assignin('base',temp_Name,zeros(Nlen,temp_Format_len));
%                 jlen=1;            
%                 for j=1:temp_Format_len
%                     temp_data(j)=typecast(uint8(tmep_dataC(jlen:jlen+temp_Format_leni(j)-1)),temp_Format_type(j));
%                     jlen=jlen+temp_Format_leni(j);
%                 end
% %                   assignin('base',temp_Name,temp_data)
% %                   evalin('base',[temp_Name{1,1} ,'(2,:)= ','temp_data',';'])
% %                  eval([temp_Name{1,1},'(2,:) = ','temp_data',';'])
%                   eval([temp_Name{1,1} ,'= [',temp_Name{1,1},';','temp_data','];'])
                i=i+temp_MsgLen;
                k=i;
            else
                i=i+1;
            end
        else
            i=i+1;
        end
    end
    temp_C(1:k-1)=[];
    u=u+1
    

end
    fclose(fileID);
    


