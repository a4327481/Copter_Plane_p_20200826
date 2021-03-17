function [attitude_error_vectoro,thrust_error_angle,attitude_target_quato]= thrust_heading_rotation_angles ( attitude_target_quat,  attitude_vehicle_quat)
 
global AC_Attitude

  p_angle_yaw                                     = AC_Attitude.p_angle_yaw;
  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        = AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS;
  
   %rotation from the target body frame to the inertial frame.
    att_to_rot_matrix=rotation_matrix(attitude_target_quat);
    att_to_thrust_vec = (att_to_rot_matrix * [0.0, 0.0, 1.0]')';

    %rotation from the current body frame to the inertial frame.
    att_from_rot_matrix=rotation_matrix(attitude_vehicle_quat);
    att_from_thrust_vec = (att_from_rot_matrix *[0.0, 0.0, 1.0]')';

    %the cross product of the desired and target thrust vector defines the rotation vector
    thrust_vec_cross = cross(att_from_thrust_vec ,att_to_thrust_vec);

    %the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_error_angle = acos(constrain_value(att_from_thrust_vec * att_to_thrust_vec', -1.0, 1.0));

   % Normalize the thrust rotation vector
     thrust_vector_length = norm(thrust_vec_cross,2);
    if ((thrust_vector_length==0 )||(thrust_error_angle==0))  
        thrust_vec_cross = [0, 0, 1];
        thrust_error_angle = 0.0;
    else  
        thrust_vec_cross =thrust_vec_cross/thrust_vector_length;
    end
   % Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat=from_axis_angle(thrust_vec_cross, thrust_error_angle);

   % Rotate thrust_vec_correction_quat to the att_from frame
    thrust_vec_correction_quat = quatmultiply(quatmultiply(quatconj(attitude_vehicle_quat) , thrust_vec_correction_quat) , attitude_vehicle_quat);

  % calculate the remaining rotation required after thrust vector is rotated transformed to the att_from frame
    yaw_vec_correction_quat = quatmultiply(quatmultiply(quatconj(thrust_vec_correction_quat) , quatconj(attitude_vehicle_quat)), attitude_target_quat);

    % calculate the angle error in x and y.   
    rotation=to_axis_angle(thrust_vec_correction_quat);
    attitude_error_vector.x = rotation(1);
    attitude_error_vector.y = rotation(2);
    %calculate the angle error in z (x and y should be zero here).
    rotation=to_axis_angle(yaw_vec_correction_quat);
    attitude_error_vector.z = rotation(3);

    % Todo: Limit roll an pitch error based on output saturation and maximum error.

    % Limit Yaw Error based on maximum acceleration - Update to include output saturation and maximum error.
    % Currently the limit is based on the maximum acceleration using the linear part of the SQRT controller.
    % This should be updated to be based on an angle limit, saturation, or unlimited based on user defined parameters.
    if ((p_angle_yaw~=0) && (abs(attitude_error_vector.z) > AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / p_angle_yaw)) 
        attitude_error_vector.z = constrain_value(wrap_PI(attitude_error_vector.z), -AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / p_angle_yaw, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS / p_angle_yaw);
        yaw_vec_correction_quat=from_axis_angle3([0.0, 0.0, attitude_error_vector.z]);
        attitude_target_quat = quatmultiply(quatmultiply(attitude_vehicle_quat , thrust_vec_correction_quat), yaw_vec_correction_quat);
    end
 
attitude_error_vectoro=[attitude_error_vector.x attitude_error_vector.y attitude_error_vector.z];
attitude_target_quato=attitude_target_quat;

  AC_Attitude.p_angle_yaw                                     = p_angle_yaw;
  AC_Attitude.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS        = AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS;
end

