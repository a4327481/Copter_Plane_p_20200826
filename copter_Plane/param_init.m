function param_init()

global plane_mode
persistent init
if isempty(init)
    init = true;
    switch plane_mode
        case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1}
            param_V10_init();
        case {ENUM_plane_mode.V1000,ENUM_plane_mode.V10s}
            param_V1000_init();
    end
end
end

