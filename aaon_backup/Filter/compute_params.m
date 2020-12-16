function ret=compute_params( sample_freq,  cutoff_freq) 
    ret.cutoff_freq = cutoff_freq;
    ret.sample_freq = sample_freq;
    ret.b0 = 0;
    ret.b1 = 0;
    ret.b2 = 0;
    ret.a1 = 0;
    ret.a2 = 0;
    if (ret.cutoff_freq<=0)     
        % zero cutoff means pass-thru
    else
    fr = sample_freq/cutoff_freq;
    ohm = tan(pi/fr);
    c = 1.0+2.0*cos(pi/4.0)*ohm + ohm*ohm;
    ret.b0 = ohm*ohm/c;
    ret.b1 = 2.0*ret.b0;
    ret.b2 = ret.b0;
    ret.a1 = 2.0*(ohm*ohm-1.0)/c;
    ret.a2 = (1.0-2.0*cos(pi/4.0)*ohm+ohm*ohm)/c;
    end
end

