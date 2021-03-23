function set_throttle_mix_max( ratio)
global throttle_rpy_mix_desired
global thr_mix_min
global thr_mix_max
    ratio = constrain_float(ratio, 0.0, 1.0);
    throttle_rpy_mix_desired = (1.0 - ratio) * thr_mix_min + ratio * thr_mix_max;
end

