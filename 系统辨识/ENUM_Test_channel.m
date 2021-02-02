classdef ENUM_Test_channel < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration
        roll_in       (0),
        pitch_in      (1),
        yaw_in        (2),
        throttle_in      (3),
        k_rudder       (4),
        k_elevator      (5),
        k_throttle       (6),
        k_aileron      (7),
        k_flap      (8),        
    end
end

