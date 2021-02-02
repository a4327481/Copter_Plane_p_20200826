classdef SpoolState < Simulink.IntEnumType
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    enumeration
        SHUT_DOWN (0),                      % all motors stop
        GROUND_IDLE (1),                    % all motors at ground idle
        SPOOLING_UP (2),                       % increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED (3),             % throttle is no longer constrained by start up procedure
        SPOOLING_DOWN (4)                     % decreasing maximum throttle while stabilizing
    end
end

