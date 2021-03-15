function output=get_filt_alpha(cutoff_freq)
global dt
    if ( cutoff_freq <= 0.0)
        output=1;
    else
        rc = 1.0/(2*pi*cutoff_freq);
        output= constrain_value(dt/(dt+rc), 0.0, 1.0);
    end
end

