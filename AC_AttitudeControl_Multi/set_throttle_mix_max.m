function set_throttle_mix_max( ratio)

global AP_Motors

thr_mix_min                      = AP_Motors.thr_mix_min;
thr_mix_max                      = AP_Motors.thr_mix_max;


    ratio = constrain_value(ratio, 0.0, 1.0);
    throttle_rpy_mix_desired = (1.0 - ratio) * thr_mix_min + ratio * thr_mix_max;

AP_Motors.throttle_rpy_mix_desired         = throttle_rpy_mix_desired;

end

