function out = deadzonef(in,dead,in_max)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明

if(abs(in)<=dead)
    out=0;
elseif(in<0)
    out=(in+dead)/(in_max-dead)*in_max;     
elseif(in>0)
    out=(in-dead)/(in_max-dead)*in_max;
else
    out=0;
end
end

