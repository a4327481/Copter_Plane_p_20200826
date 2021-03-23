function normalise_rpy_factors()
global roll_factor  
global pitch_factor
global yaw_factor
    % find maximum roll, pitch and yaw factors
    for i=1:4  
            if (roll_fac < abs(roll_factor(i)))  
                roll_fac = abs(roll_factor(i));
            end
            if (pitch_fac < abs(pitch_factor(i)))  
                pitch_fac = abs(pitch_factor(i));
            end
            if (yaw_fac < abs(yaw_factor(i)))  
                yaw_fac = abs(yaw_factor(i));
            end          
    end
    % scale factors back to -0.5 to +0.5 for each axis
    for i=1:4    
        if (roll_fac>0)
            roll_factor(i)  = 0.5 * roll_factor(i) / roll_fac;
        end
        if (pitch_fac>0)
            pitch_factor(i) = 0.5 * pitch_factor(i) / pitch_fac;
        end
        if (yaw_fac>0)
            yaw_factor(i)   = 0.5 * yaw_factor(i) / yaw_fac;
        end     
    end 
end

