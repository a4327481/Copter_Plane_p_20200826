function sample_out = LowPassFilter2p(sample,sample_freq,  cutoff_freq)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明if
persistent inint
    if isempty(inint)
        inint = 0;
    end
    if(inint==0)
    params=compute_params( sample_freq,  cutoff_freq);
    inint=1;
    end
sample_out= DigitalBiquadFilter_apply(sample, params);  
end

