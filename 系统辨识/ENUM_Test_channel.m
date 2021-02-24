classdef ENUM_Test_channel < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration
        roll_in_t       (0),
        pitch_in_t      (1),
        yaw_in_t        (2),
        throttle_in_t   (3),
        k_rudder_t      (4),
        k_elevator_t    (5),
        k_throttle_t    (6),
        k_aileron_t     (7),
        k_flap_t        (8),        
    end
end

