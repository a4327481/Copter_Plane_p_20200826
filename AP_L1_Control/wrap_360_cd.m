function  res=wrap_360_cd(  angle)
     res = mod(angle, 36000.0);
    if (res < 0) 
        res =res + 36000.0;
    end
     
end

