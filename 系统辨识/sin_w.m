function sin_w()
% 生成正弦波
global Test_w
global systime
persistent Test_Sate

if isempty(Test_Sate)
    Test_Sate = ENUM_Test_Sate.start;
end

if (Test_w.start==0)
    Test_w.out=0;
    Test_Sate=ENUM_Test_Sate.start;
else
    switch Test_Sate
        
        case ENUM_Test_Sate.start
            Test_w.out=0;
            Test_Sate=ENUM_Test_Sate.run;
            Test_w.t0=systime;
            Test_w.fai0=0;
            Test_w.fai=0;
            Test_w.dt=0;
            Test_w.i=1;
            Test_w.w=Test_w.ws;
        case ENUM_Test_Sate.run
            temp_w_nT=Test_w.n*1/Test_w.w;
            Test_w.dt=systime-Test_w.t0;
            if(Test_w.dt<temp_w_nT)
                Test_w.fai=Test_w.fai0+Test_w.w*2*pi*(systime-Test_w.t0);
                Test_w.out=sin(Test_w.fai)*Test_w.Amp;
            elseif(Test_w.w+Test_w.dw<Test_w.we)
                Test_w.fai=Test_w.fai0+Test_w.w*2*pi*(systime-Test_w.t0);
                Test_w.out=sin(Test_w.fai)*Test_w.Amp;
                Test_w.fai0=Test_w.fai;
                Test_w.t0=systime;
                Test_w.w=Test_w.w+Test_w.dw;
            elseif(Test_w.w+Test_w.dw<Test_w.we+Test_w.dw)
                Test_w.fai=Test_w.fai0+Test_w.w*2*pi*(systime-Test_w.t0);
                Test_w.out=sin(Test_w.fai)*Test_w.Amp;
                Test_w.fai0=Test_w.fai;
                Test_w.t0=systime;
                Test_w.w=Test_w.w+Test_w.dw;
            else
                Test_w.out=0;
                Test_Sate=ENUM_Test_Sate.idle;
            end
        case ENUM_Test_Sate.idle
            Test_w.out=0;
            
    end
end
end


