function init_setup_motors()
%初始化电动动力分配
global plane_mode
persistent inint
if isempty(inint)
    inint = 1;
end
if(inint)
    switch plane_mode
        case {ENUM_plane_mode.V10 ,ENUM_plane_mode.V10_1}
            setup_motors_4a1() ;
            inint=0;
        case {ENUM_plane_mode.V10s,ENUM_plane_mode.V1000}
            setup_motors() ;
            inint=0;
    end
end
end

