function  yawo=get_yaw()
 global AP_L1
 global SINS
 
 yaw            = SINS.yaw;
 reverse=AP_L1.reverse;
       yawo=yaw;
    if (reverse)  
       yawo=wrap_PI(pi +yaw);
    end
end

