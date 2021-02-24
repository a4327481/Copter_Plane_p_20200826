function sin_step_test()
%正弦和阶跃信号测试
global Test_w
        switch Test_w.mode
            case ENUM_Test_mode.step_w
                step_w();
            case ENUM_Test_mode.sin_w
                sin_w();
        end
    Test_w.roll_in=(Test_w.channel==ENUM_Test_channel.roll_in_t)*Test_w.out;
    Test_w.pitch_in=(Test_w.channel==ENUM_Test_channel.pitch_in_t)*Test_w.out;
    Test_w.yaw_in=(Test_w.channel==ENUM_Test_channel.yaw_in_t)*Test_w.out;
    Test_w.throttle_in=(Test_w.channel==ENUM_Test_channel.throttle_in_t)*Test_w.out;
    Test_w.k_rudder=(Test_w.channel==ENUM_Test_channel.k_rudder_t)*Test_w.out;
    Test_w.k_elevator=(Test_w.channel==ENUM_Test_channel.k_elevator_t)*Test_w.out;
    Test_w.k_throttle=(Test_w.channel==ENUM_Test_channel.k_throttle_t)*Test_w.out;
    Test_w.k_aileron=(Test_w.channel==ENUM_Test_channel.k_aileron_t)*Test_w.out;
    Test_w.k_flap=(Test_w.channel==ENUM_Test_channel.k_flap_t)*Test_w.out;   
end

