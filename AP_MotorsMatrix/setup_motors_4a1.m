function setup_motors_4a1() 
    % MOTOR_FRAME_TYPE_H:
    % H frame set-up - same as X but motors spin in opposite directiSons
    global AP_MOTORS_MATRIX_YAW_FACTOR_CW
    global AP_MOTORS_MATRIX_YAW_FACTOR_CCW
    
    add_motor(1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
    add_motor(2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
    add_motor(3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
    add_motor(4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
end

