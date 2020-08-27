function [CMD, k] = read_CMD(C) %#codegen
len=length(C);
i=1;
k=1;
CMD.time=zeros(len,1);
CMD.command_total=zeros(len,1);
CMD.sequnce=zeros(len,1);
CMD.command=zeros(len,1);
CMD.param1=zeros(len,1);
CMD.param2=zeros(len,1);
CMD.param3=zeros(len,1);
CMD.param4=zeros(len,1);
CMD.latitude=zeros(len,1);
CMD.longitude=zeros(len,1);
CMD.altitude=zeros(len,1);
while(i<len)
    if (C(i)==hex2dec('db')&&C(i+1)==hex2dec('90')&&C(i+2)==hex2dec('20'))
       CMD.time(k)=typecast(uint8(C(i+3:i+6)),'uint32');
       CMD.command_total(k)=typecast(uint8(C(i+7:i+8)),'uint16');
       CMD.sequnce(k)=typecast(uint8(C(i+9:i+10)),'uint16');
       CMD.command(k)=typecast(uint8(C(i+11:i+12)),'uint16');
       CMD.param1(k)=typecast(uint8(C(i+13:i+16)),'single');
       CMD.param2(k)=typecast(uint8(C(i+17:i+20)),'single');
       CMD.param3(k)=typecast(uint8(C(i+21:i+24)),'single');
       CMD.param4(k)=typecast(uint8(C(i+25:i+28)),'single');
       CMD.latitude(k)=typecast(uint8(C(i+29:i+32)),'single');
       CMD.longitude(k)=typecast(uint8(C(i+33:i+36)),'single');
       CMD.altitude(k)=typecast(uint8(C(i+37:i+40)),'single');
       i=i+40;
       k=k+1;
    else
        i=i+1;
    end
      
end
% CMD.time(k:len)=[];
% CMD.command_total(k:len)=[];
% CMD.sequnce(k:len)=[];
% CMD.command(k:len)=[];
% CMD.param1(k:len)=[];
% CMD.param2(k:len)=[];
% CMD.param3(k:len)=[];
% CMD.param4(k:len)=[];
% CMD.latitude(k:len)=[];
% CMD.longitude(k:len)=[];
% CMD.altitude(k:len)=[];