
%V1000 to txt  
% Display surf plot of the currently selected data.
%mat 鏍煎紡杞崲涓虹敾鍥?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%open and read
global PathName
if PathName~=0
    cd(PathName);
    [FileName,PathName,~] = uigetfile([PathName,'\\*.*']); % 璇诲彇闆疯揪鏁版嵁
else
    [FileName,PathName,~] = uigetfile('*.*'); % 璇诲彇闆疯揪鏁版嵁
end
if FileName==0
    return;
end
BLOCK_SIZE=296;%288%296

fp=fopen([PathName,'\\',FileName],'r');
        data = fread(fp);
        fclose(fp);
        n=length(data);
        i=1;
        m=floor(n/BLOCK_SIZE);
        data=reshape(data(1:BLOCK_SIZE*m)',[BLOCK_SIZE,m]);
        data=data';
        
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
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        len_m=floor(m/32);
        ID=zeros(len_m,255);      
        temp=reshape([data(:,247:248)'],1,[]);
        snr=double(typecast(uint8(temp),'uint16')');
        temp=reshape([data(:,249:250)'],1,[]);        
        id=typecast(uint8(temp),'uint16')';
        len=length(id);
        for i=1:len_m*32
        %GPS
            i_32=floor(i/32)+1;
            if(id(i)~=0)
            ID(i_32,id(i))=snr(i); 
            end
        end
          len=max(size(ID));        
          t=(1:len)*0.004*32;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ID_is_mun=zeros(1,255);
%         ID_is    =zeros(1,255);
        for j=1:255
            ID_is_mun(j)=sum((ID(:,j)~=0));
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        del_num=0.9;%鏈夋晥鏁版嵁鐧惧垎姣?
        id_plot=1:32;%缁樺浘鑼冨洿GLO 65-96，193-197，BD 33-64，159-163
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ID_is=ID_is_mun>del_num*len_m;
        num_is=find(ID_is(id_plot)==1);   
        plot(t,ID(:,id_plot(num_is)));



















