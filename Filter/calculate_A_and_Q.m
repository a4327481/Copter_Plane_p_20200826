function  [A,   Q]=calculate_A_and_Q(  center_freq_hz,   bandwidth_hz,   attenuation_dB) 
    A = power(10, -attenuation_dB / 40.0);
    if (center_freq_hz > 0.5 * bandwidth_hz) 
         octaves = log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0)) * 2.0;
        Q = sqrt(power(2, octaves)) / (power(2, octaves) - 1.0);
     else 
        Q = 0.0;
    end
    


