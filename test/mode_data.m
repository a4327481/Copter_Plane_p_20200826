Aero.Sref=1.124;
Aero.Lref=0.3114;
Aero.Jx=8.07546425;
Aero.Jy=4.17029;
Aero.Jz=4.010776893;
Aero.Tmax=27*9.8*0.5;
HD=180/pi;

Aero.CL=[
0.767869272
1.095366185
1.409441063
1.682768829
1.853098264
1.897922071];

Aero.CL_alpha=[
-3
0
3
6
9
12];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Aero.CD=[    
0.068676435
0.079381248
0.100082439
0.127579621
0.163397582
0.213587951];

Aero.CD_alpha=[
-3
0
3
6
9
12];

Aero.CC=[ 
    0
0.018122654
0.036541923
0.050539414];

Aero.CC_beta=[
    0
    3
    6
    9];
Aero.CC_dr=0.029836112/15;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cm_data=[
0.001889435
0.22380212
-0.115666
-0.047363078
0.062662 
0.001186697
0.17396559
-0.090838168
-0.016074557
0.068240 
0.000494569
0.1036226
-0.067919388
0.014645537
0.050843 
-0.000262666
0.023816853
-0.045944899
0.044563239
0.022173 
-0.000963774
-0.064354702
-0.028005074
0.074650331
-0.018673 
-0.001388939
-0.16231179
-0.021700888
0.101139157
-0.084262]; 

Aero.Cm_alpha=[
-3
0
3
6
9
12];
% %
% F-机身
% H-平尾
% L-机翼
% V-立尾
Cm_data_all=reshape(Cm_data,5,[])';
Aero.Cm_V=Cm_data_all(:,1);
Aero.Cm_H=Cm_data_all(:,2);
Aero.Cm_L=Cm_data_all(:,3);
Aero.Cm_F=Cm_data_all(:,4);
Aero.Cm_sum=Cm_data_all(:,5);
% hold on
% plot(Aero.Cm_alpha,Aero.Cm_V);
% plot(Aero.Cm_alpha,Aero.Cm_H);
% plot(Aero.Cm_alpha,Aero.Cm_L);
% plot(Aero.Cm_alpha,Aero.Cm_F);
% plot(Aero.Cm_alpha,Aero.Cm_sum);
% legend('V','H','L','F','sum')


Cm_data_elevator=[
-0.10514733	-0.10499458	-0.10605265	-0.105726028	-0.10810534	-0.1076928
-0.13760585	-0.033901441	0.073374812	0.17396559	0.27718708	0.37578667]';
Cm_H_elevator=Cm_data_elevator(:,2);
Cm_sum_del_H =Cm_data_elevator(:,1);
Aero.Cm_elevator=[-15 -10 -5 0 5 10];
Aero.Cm_H_de=Cm_H_elevator-Cm_H_elevator(4);
% plot(Aero.Cm_elevator,Aero.Cm_H_de)
% legend('Cm_H_elevator')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Aero.beta=[0;3;6;9];
Aero.Cl_beta=[
    0
    -0.047535999
    -0.10153719
    -0.14804008
    ];
Aero.Cm_beta=[
    0
    0.052917889
    0.052163516
    0.034450043
    ];
Aero.Cn_beta=[
    0
    0.034639977
    0.075946103
    0.10263015 
    ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Aero.Cn_dr=0.06623314/15;% rudder =[0 30]
Aero.Cn_da=-0.065367069/15;
Aero.Cl_da= 1.035848963/15;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vehicle.Motor.commandToW2Gain=100;
Vehicle.Rotor.lock           =7.976243528;
Vehicle.Rotor.thetaTip       =13.48/HD;
Vehicle.Rotor.radius         =0.3302;
Vehicle.Rotor.area           =pi*Vehicle.Rotor.radius^2;
Vehicle.Airframe.xy          =0.5;
Vehicle.Airframe.h           =0 ;
Vehicle.Rotor.Ct             =1 ;
Vehicle.Rotor.Cq             =1 ;






