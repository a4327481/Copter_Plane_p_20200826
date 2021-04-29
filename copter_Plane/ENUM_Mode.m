classdef ENUM_Mode < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration 
        MANUAL              (0),
        Copter_STABILIZE    (1),
        Copter_ALT_HOLD     (2),
        Copter_POS_HOLD     (3),
        Plane_STABILIZE     (4),
        Plane_TECS          (5),
        Plane_L1_WAYPOINT   (6),
        Copter_Plane_MANUAL (7),
        Plane_L1_LOITER     (8),
        AUTO_TEST           (9),
        AUTO                (10),
    end
end

