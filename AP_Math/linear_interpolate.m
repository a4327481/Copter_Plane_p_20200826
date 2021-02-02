function output=linear_interpolate( low_output,  high_output,var_value,var_low,  var_high)
% /*
%  * linear interpolation based on a variable in a range
%  */
    if (var_value <= var_low) 
        output=low_output;
        return 
    end
    if (var_value >= var_high) 
        output=high_output;
        return 
    end
     p = (var_value - var_low) / (var_high - var_low);
     output=low_output + p * (high_output - low_output);
end

