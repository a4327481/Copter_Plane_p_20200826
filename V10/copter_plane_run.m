function copter_plane_run()
global plane_mode
    switch plane_mode
        case plane_mode==ENUM_plane_mode.V1000
            run_V1000();
        case plane_mode==ENUM_plane_mode.V10
            run_V10();
        case plane_mode==ENUM_plane_mode.V10s
            run_V10s();
        otherwise
            run_V1000();
    end

    
end

