function out = deadzonef(in,dead,in_max)

in=constrain_value(in,-in_max,in_max);
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

