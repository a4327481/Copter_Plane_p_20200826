function  ret=NotchFilter_init( sample_freq_hz,  center_freq_hz, bandwidth_hz,  attenuation_dB)
    % check center frequency is in the allowable range
    if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq_hz)) 
        [A, Q] = calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB);
        ret=init_with_A_and_Q(  sample_freq_hz,   center_freq_hz,   A,   Q);    
    else
        ret.b0 = 0;
        ret.b1 = 0;
        ret.b2 = 0;
        ret.a0_inv = 0;
        ret.a1 =  0;
        ret.a2 =  0; 
        ret.init=false;
    end
end

