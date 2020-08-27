function sample_out= DigitalBiquadFilter_apply(sample, params)  
persistent delay_element_1
persistent delay_element_2
    if isempty(delay_element_1)
        delay_element_1 = 0;
    end
    if isempty(delay_element_2)
        delay_element_2 = 0;
    end
    
    if(is_zero(params.cutoff_freq) || is_zero(params.sample_freq))  
        sample_out= sample; 
        return;
    end

     delay_element_0 = sample - delay_element_1 * params.a1 - delay_element_2 * params.a2;
     sample_out = delay_element_0 * params.b0 + delay_element_1 * params.b1 + delay_element_2 * params.b2;

    delay_element_2 = delay_element_1;
    delay_element_1 = delay_element_0;
end

