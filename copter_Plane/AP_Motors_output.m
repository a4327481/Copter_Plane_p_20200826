function AP_Motors_output()
%电机动力分配和输出设置。
global plane_mode
global Copter_Plane
init_setup_motors();
switch plane_mode
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%===================%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1,ENUM_plane_mode.V10s}
        AP_MotorsMulticopter_output_4a1();
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%===================%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case ENUM_plane_mode.V1000
        switch Copter_Plane.State
            case ENUM_State.E_Copter
                AP_MotorsMulticopter_output();
            case ENUM_State.E_Plane
                output_to_motors_plane();
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%===================%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

end

