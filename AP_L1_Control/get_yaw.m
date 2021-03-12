function  yawo=get_yaw()
 global yaw
 global AP_L1
 
 reverse=AP_L1.reverse;
       yawo=yaw;
    if (reverse)  
       yawo=wrap_PI(pi +yaw);
    end
end

