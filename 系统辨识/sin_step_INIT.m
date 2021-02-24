    load  bus_test_w.mat
    Test_w.start=0;
    Test_w.ws=2;
    Test_w.we=2;
    Test_w.dw=1;
    Test_w.n=5;
    Test_w.Amp=0.05;
    Test_w.offset=0.05;
    Test_w.mode=ENUM_Test_mode.step_w;
    Test_w.channel=ENUM_Test_channel.roll_in_t;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Test_w.roll_in=0;
    Test_w.pitch_in=0;
    Test_w.yaw_in=0;
    Test_w.throttle_in=0;
    Test_w.k_rudder=0;
    Test_w.k_elevator=0;
    Test_w.k_throttle=0;
    Test_w.k_aileron=0;
    Test_w.k_flap=0;      
    %%%%%%%%%%%%%%%%%%%%%%
    Test_w.w=0;
    Test_w.i=1;
    Test_w.out=0;
    Test_w.t0=0;
    Test_w.dt=0;
    Test_w.fai0=0;
    Test_w.fai=0;    
    systime=0;
%     temp_w.w=Test_w.start:Test_w.dw:Test_w.end;
%     temp_w.nT=Test_w.n*1./temp_w.w;
    


        

