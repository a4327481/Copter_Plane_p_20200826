function   ret = init_with_A_and_Q(  sample_freq_hz,   center_freq_hz,   A,   Q)

    if ((center_freq_hz > 0.0) && (center_freq_hz < 0.5 * sample_freq_hz) && (Q > 0.0)) 
         omega = 2.0 * pi * center_freq_hz / sample_freq_hz;
         alpha = sin(omega) / (2 * Q);
        ret.b0 =  1.0 + alpha*(A*A);
        ret.b1 = -2.0 * cos(omega);
        ret.b2 =  1.0 - alpha*(A*A);
        ret.a0_inv =  1.0/(1.0 + alpha);
        ret.a1 = -2.0 * cos(omega);
        ret.a2 =  1.0 - alpha; 
        ret.init=true;
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

