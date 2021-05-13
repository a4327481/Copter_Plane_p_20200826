function out_filePath=data_read_V10()
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
        Name_temp(j,1)=string([char(temp_Name1),'_temp']);
        temp_Format1=temp_Format(temp_Format~=0);
        Format_len(j,1)=uint8(length(temp_Format1));
        Format(j,1)=string(char(temp_Format1));
        for il=1:Format_len(j,1)
            [out ,outp,lenj] = Format_to_type(Format{j,1}(il));
            Format_type(j,il)=string(out);
            Format_leni(j,il)=uint8(lenj);
        end
        temp_Lables1=temp_Lables(temp_Lables~=0);
        Lables{j,1}=char(temp_Lables1);
        temp_label=string(char(temp_Lables1));
        temp_label=(strsplit(temp_label,','))';
        temp_label=cellstr(temp_label);
        temp=[{'LineNo'};temp_label];
        eval([char(temp_Name1),'_label=temp;'])
%         assignin('base',[char(temp_Name1),'_label'],[{'LineNo'};temp_label])
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
%%%%%%%%%%%%%%预处理数据，便于分配空间%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=1;
N=1e8;
temp_N=N;
u=1;
C=uint8(zeros(N,1));
while (temp_N==N)
    [C ,temp_N]= fread(fileID,N,'uint8');
    C=uint8(C);
    temp_C = [temp_C;C];
    C=[];
    len=length(temp_C);
    i=1;
    while(i<=len-max(MsgLen))
        if((temp_C(i)==163)&&(temp_C(i+1)==149))
            temp_MsgID=find(MsgID==temp_C(i+2),1);
            if(temp_MsgID)
                Name_len(temp_MsgID)=Name_len(temp_MsgID)+1;
                temp_MsgLen=MsgLen(temp_MsgID);
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
temp_C=[];
fclose(fileID);

len=length(Name);
temp_data=cell(len,1);
for i=1:len
    if(Name_len(i))
        eval([char(Name(i)),'=zeros(Name_len(i),Format_len(i)+1);'])
%         assignin('base',[char(Name(i))],(zeros(Name_len(i),Format_len(i)+1)))
        temp_data{i}=uint8(zeros(Name_len(i),MsgLen(i)-3));    
    end 
end
%%%%%%%%%%%%%%处理数据，存储至分配空间%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=1;
N=1e8;
temp_N=N;
u=1;
fileID = fopen(filePath);
while (temp_N==N)
    [C ,temp_N]= fread(fileID,N,'uint8');
    C=uint8(C);
    temp_C = [temp_C;C];
    C=[];
    len=length(temp_C);
    i=1;
    while(i<=len-max(MsgLen))
        if((temp_C(i)==163)&&(temp_C(i+1)==149))
            temp_MsgID=find(MsgID==temp_C(i+2),1);
            if(temp_MsgID)
                temp_MsgLen=MsgLen(temp_MsgID);
                temp_data{temp_MsgID,1}(Name_len_k(temp_MsgID),:)=temp_C(i+3:i+temp_MsgLen-1)';
                Name_len_k(temp_MsgID)=Name_len_k(temp_MsgID)+1;               
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
temp_C=[];
fclose(fileID);
%%%%%%%%%%%%%%解析存储空间的数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

len=length(Name);
save([filePath(1:end-4),'.mat'],Name(temp_MsgID));
for i=1:len
    if(Name_len(i))
        jlen=1;
        for j=1:Format_len(i)
            data_read=(temp_data{i,1}(:,jlen:jlen+Format_leni(i,j)-1))';
            eval([Name{i,1},'(:,j+1)=typecast(reshape(data_read,1,[]),Format_type(i,j));'])
%             evalin('base',[Name{i,1},'(:,j+1)=typecast(reshape(data_read,1,[]),Format_type(i,j));'])
            jlen=jlen+Format_leni(i,j)
        end
        save([filePath(1:end-4),'.mat'],Name(i),'-append');
        save([filePath(1:end-4),'.mat'],[char(Name(i)),'_label'],'-append');   
    end 
end
out_filePath=[filePath(1:end-4),'.mat'];
