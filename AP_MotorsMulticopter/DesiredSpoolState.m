classdef DesiredSpoolState < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration
        SHUT_DOWN (0), % all motors should move to stop
        GROUND_IDLE (1), % all motors should move to ground idle
        THROTTLE_UNLIMITED (2),%motors should move to being a state where throttle is unconstrained (e.g. by start up procedure)
    end
end