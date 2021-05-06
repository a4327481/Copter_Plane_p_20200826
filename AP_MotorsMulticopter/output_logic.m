function  output_logic()

global dt
global Copter_Plane
global AP_Motors

armed                                = Copter_Plane.armed;
disarm_disable_pwm               = AP_Motors.disarm_disable_pwm;
disarm_safe_timer                = AP_Motors.disarm_safe_timer;
safe_time                        = AP_Motors.safe_time;
spool_desired                    = AP_Motors.spool_desired;
spool_state                      = AP_Motors.spool_state;
spool_up_time                    = AP_Motors.spool_up_time;

limit_roll                      = AP_Motors.limit_roll;
limit_pitch                     = AP_Motors.limit_pitch;
limit_yaw                       = AP_Motors.limit_yaw;
limit_throttle_lower            = AP_Motors.limit_throttle_lower;
limit_throttle_upper            = AP_Motors.limit_throttle_upper;

spin_up_ratio                    = AP_Motors.spin_up_ratio;
throttle_thrust_max              = AP_Motors.throttle_thrust_max;
thrust_boost                     = AP_Motors.thrust_boost;
thrust_boost_ratio               = AP_Motors.thrust_boost_ratio;
spin_arm                         = AP_Motors.spin_arm;
spin_min                         = AP_Motors.spin_min;
throttle_filter                  = AP_Motors.throttle_filter;
thrust_balanced                  = AP_Motors.thrust_balanced;

    if (armed) 
        if (disarm_disable_pwm && (disarm_safe_timer < safe_time)) 
            disarm_safe_timer=disarm_safe_timer + dt;
         else 
            disarm_safe_timer = safe_time;
        end    
     else 
           disarm_safe_timer = 0.0;
    end

    % force desired and current spool mode if disarmed or not interlocked
    if (~armed) 
        spool_desired = SpoolState.SHUT_DOWN;
        spool_state = SpoolState.SHUT_DOWN;
    end
    
    if (spool_up_time < 0.05) 
        % prevent float exception
        spool_up_time=0.05;
    end
    

    switch (spool_state)
        case SpoolState.SHUT_DOWN
            % Motors should be stationary.
            % Servos set to their trim values or in a test condition.
            
            % set limits flags
            limit_roll = true;
            limit_pitch = true;
            limit_yaw = true;
            limit_throttle_lower = true;
            limit_throttle_upper = true;
            
            % make sure the motors are spooling in the correct direction
            if (spool_desired ~= SpoolState.SHUT_DOWN && disarm_safe_timer >= safe_time)
                spool_state = SpoolState.GROUND_IDLE;
            end
                % set and increment ramp variables
                spin_up_ratio = 0.0;
                throttle_thrust_max = 0.0;
               
                % initialise motor failure variables
                thrust_boost = false;
                thrust_boost_ratio = 0.0;
            
        case SpoolState.GROUND_IDLE
            % Motors should be stationary or at ground idle.
            % Servos should be moving to correct the current attitude.
            
            % set limits flags
            limit_roll = true;
            limit_pitch = true;
            limit_yaw = true;
            limit_throttle_lower = true;
            limit_throttle_upper = true;
            
            % set and increment ramp variables
            spool_step = dt / (spool_up_time );
            switch (spool_desired)
                case SpoolState.SHUT_DOWN
                    spin_up_ratio=spin_up_ratio - spool_step;
                    % constrain ramp value and update mode
                    if (spin_up_ratio <= 0.0)
                        spin_up_ratio = 0.0;
                        spool_state = SpoolState.SHUT_DOWN;     
                    end
                case SpoolState.THROTTLE_UNLIMITED
                    spin_up_ratio=spin_up_ratio + spool_step;
                    % constrain ramp value and update mode
                    if (spin_up_ratio >= 1.0)
                        spin_up_ratio = 1.0;
                        spool_state = SpoolState.SPOOLING_UP;
                    end
                case SpoolState.GROUND_IDLE
                    spin_up_armed_ratio = 0.0;
                    if (spin_min > 0.0)
                        spin_up_armed_ratio = spin_arm / spin_min;
                    end
                    spin_up_ratio=spin_up_ratio + constrain_value(spin_up_armed_ratio - spin_up_ratio, -spool_step, spool_step);
            end
            throttle_thrust_max = 0.0;
            
            % initialise motor failure variables
            thrust_boost = false;
            thrust_boost_ratio = 0.0;
    
    case SpoolState.SPOOLING_UP
        % Maximum throttle should move from minimum to maximum.
        % Servos should exhibit normal flight behavior.

        % initialize limits flags
        limit_roll = false;
        limit_pitch = false;
        limit_yaw = false;
        limit_throttle_lower = false;
        limit_throttle_upper = false;

        % make sure the motors are spooling in the correct direction
        if (spool_desired ~=SpoolState.THROTTLE_UNLIMITED) 
            spool_state = SpoolState.SPOOLING_DOWN;
%             return;
        end
        % set and increment ramp variables
        spin_up_ratio = 1.0;
        throttle_thrust_max=throttle_thrust_max + 1.0 / (spool_up_time * (1/dt));
        % constrain ramp value and update mode
        if (throttle_thrust_max >= min(throttle_filter, get_current_limit_max_throttle())) 
            throttle_thrust_max = get_current_limit_max_throttle();
            spool_state = SpoolState.THROTTLE_UNLIMITED;
        elseif (throttle_thrust_max < 0.0) 
            throttle_thrust_max = 0.0;      
        end
        % initialise motor failure variables
        thrust_boost = false;
        thrust_boost_ratio = max(0.0, thrust_boost_ratio - 1.0 / (spool_up_time * (1/dt)));
    case SpoolState.THROTTLE_UNLIMITED
        % Throttle should exhibit normal flight behavior.
        % Servos should exhibit normal flight behavior.

        % initialize limits flags
        limit_roll = false;
        limit_pitch = false;
        limit_yaw = false;
        limit_throttle_lower = false;
        limit_throttle_upper = false;

        % make sure the motors are spooling in the correct direction
        if (spool_desired ~= SpoolState.THROTTLE_UNLIMITED) 
            spool_state = SpoolState.SPOOLING_DOWN;
%             return;
        end
        % set and increment ramp variables
        spin_up_ratio = 1.0;
        throttle_thrust_max = get_current_limit_max_throttle();

        if (thrust_boost && ~thrust_balanced) 
            thrust_boost_ratio = min(1.0, thrust_boost_ratio + 1.0 / (spool_up_time * (1/dt)));
         else 
            thrust_boost_ratio = max(0.0, thrust_boost_ratio - 1.0 / (spool_up_time * (1/dt))); 
        end
    case SpoolState.SPOOLING_DOWN
        % Maximum throttle should move from maximum to minimum.
        % Servos should exhibit normal flight behavior.

        % initialize limits flags
        limit_roll = false;
        limit_pitch = false;
        limit_yaw = false;
        limit_throttle_lower = false;
        limit_throttle_upper = false;

        % make sure the motors are spooling in the correct direction
        if (spool_desired == SpoolState.THROTTLE_UNLIMITED) 
            spool_state = SpoolState.SPOOLING_UP;
%             return;
        end

        % set and increment ramp variables
        spin_up_ratio = 1.0;
        throttle_thrust_max=throttle_thrust_max - 1.0 / (spool_up_time * (1/dt));

        % constrain ramp value and update mode
        if (throttle_thrust_max <= 0.0) 
            throttle_thrust_max = 0.0;
        end
        if (throttle_thrust_max >= get_current_limit_max_throttle()) 
            throttle_thrust_max = get_current_limit_max_throttle();
        elseif (is_zero(throttle_thrust_max)) 
            spool_state = SpoolState.GROUND_IDLE;
        end 
        thrust_boost_ratio = max(0.0, thrust_boost_ratio - 1.0 / (spool_up_time * (1/dt)));
    end

AP_Motors.disarm_disable_pwm               = disarm_disable_pwm;
AP_Motors.disarm_safe_timer                = disarm_safe_timer;
AP_Motors.safe_time                        = safe_time;
AP_Motors.spool_desired                    = spool_desired;
AP_Motors.spool_state                      = spool_state;
AP_Motors.spool_up_time                    = spool_up_time;
AP_Motors.spin_up_ratio                    = spin_up_ratio;
AP_Motors.throttle_thrust_max              = throttle_thrust_max;
AP_Motors.thrust_boost                     = thrust_boost;
AP_Motors.thrust_boost_ratio               = thrust_boost_ratio;
AP_Motors.spin_arm                         = spin_arm;
AP_Motors.spin_min                         = spin_min;
AP_Motors.throttle_filter                  = throttle_filter;
AP_Motors.thrust_balanced                  = thrust_balanced;

 AP_Motors.limit_roll                      = limit_roll;
 AP_Motors.limit_pitch                     = limit_pitch;
 AP_Motors.limit_yaw                       = limit_yaw ;
 AP_Motors.limit_throttle_lower            = limit_throttle_lower;
 AP_Motors.limit_throttle_upper            = limit_throttle_upper;

end

