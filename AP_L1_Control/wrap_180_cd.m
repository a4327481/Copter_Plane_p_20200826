function res=wrap_180_cd(angle)

     res = wrap_360_cd(angle);
    if (res > 18000) 
        res =res- 36000;
    end
end

