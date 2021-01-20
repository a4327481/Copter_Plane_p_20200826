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
RPM_N_Nm_20_15_20m_s=[
3009	5	0.39
3308	10	0.68
3599	15	0.94
3866	20	1.18
4124	25	1.41
4371	30	1.63
4588	35	1.84
4818	40	2.04
5040	45	2.24
5250	50	2.43
5455	55	2.62
5653	60	2.81];
Rotor_2015.w=RPM_N_Nm_20_15_20m_s(:,1)*2*pi/60;
Rotor_2015.F=RPM_N_Nm_20_15_20m_s(:,2);
Rotor_2015.M=RPM_N_Nm_20_15_20m_s(:,3);

RPM_N_Nm_26_8_0m_s=[
575	0.13	0.05
876	0.33	0.11
999	0.44	0.15
1108	0.55	0.18
1298	0.77	0.25
1461	1	0.32
1468	1.02	0.33
1611	1.23	0.39
1740	1.44	0.46
1861	1.64	0.52
1970	1.85	0.58
2072	2.04	0.64
2191	2.33	0.73
2280	2.53	0.79
2331	2.62	0.82
2495	3.02	0.94
2648	3.44	1.08
2776	3.71	1.16
2796	3.78	1.18
2929	4.19	1.32
3059	4.61	1.44
3134	4.82	1.51
3136	4.79	1.5
3138	4.83	1.5
3248	5.2	1.6
3658	6.71	2.06
4070	8.46	2.6
4460	10.09	3.14
4826	12.12	3.77
5132	13.75	4.32
5464	15.88	5.01];
Rotor_2608.w=RPM_N_Nm_26_8_0m_s(:,1)*2*pi/60;
Rotor_2608.F=RPM_N_Nm_26_8_0m_s(:,2);
Rotor_2608.M=RPM_N_Nm_26_8_0m_s(:,3);

Thr_RPM_N_Nm_26_8_0m_s_f=[
35.9 	2607 	3119 	1.178 
37.9 	2802 	3647 	1.361 
41.1 	2998 	4167 	1.565 
44.9 	3201 	4780 	1.785 
48.7 	3399 	5406 	2.006 
53.5 	3602 	6072 	2.269]; 

Rotor_2608_f.Thr=Thr_RPM_N_Nm_26_8_0m_s_f(:,1)/100;
Rotor_2608_f.w  =Thr_RPM_N_Nm_26_8_0m_s_f(:,2)*2*pi/60;
Rotor_2608_f.F  =Thr_RPM_N_Nm_26_8_0m_s_f(:,3)*9.8/1000;
Rotor_2608_f.M  =Thr_RPM_N_Nm_26_8_0m_s_f(:,4);
Rotor_2608_f.rho= 1.225*288.15*103730/(273.15+14.9)/101325;
Rotor_2608_f.radius=26*0.01*2.54/2;
Rotor_2608_f.area=Rotor_2608_f.radius^2*pi;
Rotor_2608_f.Ctf=Rotor_2608_f.F./(Rotor_2608_f.w.^2*Rotor_2608_f.rho*Rotor_2608_f.area*Rotor_2608_f.radius^2);
Rotor_2608_f.Ct=mean(Rotor_2608_f.Ctf);
Rotor_2608_f.Cqf=Rotor_2608_f.M./(Rotor_2608_f.w.^2*Rotor_2608_f.rho*Rotor_2608_f.area*Rotor_2608_f.radius^3);
Rotor_2608_f.Cq=mean(Rotor_2608_f.Cqf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vehicle.Motor.commandToW2Gain=100;
Vehicle.Rotor.lock           =7.976243528;
Vehicle.Rotor.thetaTip       =13.48/HD;
Vehicle.Rotor.radius         =Rotor_2608_f.radius;
Vehicle.Rotor.area           =Rotor_2608_f.area;
Vehicle.Airframe.xy          =0.5;
Vehicle.Airframe.h           =0 ;
Vehicle.Rotor.Ct             =Rotor_2608_f.Ct;
Vehicle.Rotor.Cq             =Rotor_2608_f.Cq ;