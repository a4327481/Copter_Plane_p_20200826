classdef ENUM_Test_channel < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration

        roll_target_t           (0)
        pitch_target_t          (1)
        target_yaw_rate_t       (2)
        rate_target_ang_vel0_t  (3)
        rate_target_ang_vel1_t  (4)
        rate_target_ang_vel2_t  (5)      
        roll_in_t               (6),
        pitch_in_t              (7),
        yaw_in_t                (8),
        throttle_in_t           (9),
        k_rudder_t              (10),
        k_elevator_t            (11),
        k_throttle_t            (12),
        k_aileron_t             (13),
        k_flap_t                (14),
    end
end

