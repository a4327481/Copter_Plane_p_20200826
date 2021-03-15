function rate_target_to_motor_pitch( target,  measurement,  limit)
 
    % don't process inf or NaN
_flags_reset_filter=

    % reset input filter to value received
    if (_flags_reset_filter)  
        _flags_reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
      else  
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);

        % calculate and filter derivative
        if (_dt > 0.0f)  
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
         
     

    % update I term
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    % calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier(_pid_info.P + _pid_info.D, _dt);

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
 
end

