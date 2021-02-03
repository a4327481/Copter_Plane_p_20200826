
%%%%%%%%%%%%%%%%%%%%
% mat_normal
%%%%%%%%%%%%%%%%%%
a11=0;
a12=0;
a13=0;
a21=0;
a22=0;
a23=0;
a24=0;
a41=0;
a42=0;
a44=0;

b11=1;
b12=1;
b21=1;
b22=1;
b41=1;
b42=1;
A=[a11 a12 a13 0;a21 a22 a23 a24;0 0 0 1;a41 a42 0 a44];
B = [b11 b12;b21 b22;0 0;b41 b42];
% A=out_f.A;
% B=out_f.B;
C = [1 0 0 0;0 0 1 0;0 0 0 1]; 
% C = [1 0 0 0;0 0 1 0]; 
D = 0; 
K = zeros(4,3);

Ve=17;
V_f1=V_f-Ve;
% x0=[V_f1(1) pitch_f(1) pitch_f(1) wz_f(1)]';
x0=[V_f1(1) pitch_f(1) pitch_f(1) wz_f(1)]';
Ts = 0;
plan_e = idss(A,B,C,D,K,x0,Ts);
S_plan_e = plan_e.Structure; 
S_plan_e.A.Free=[1 1 1 0;1 1 1 1;0 0 0 0;1 1 0 1];
S_plan_e.B.Free=[1 1;1 1;0 0;1 1];
S_plan_e.C.Free=zeros(3,4);
S_plan_e.D.Free=zeros(3,2);
S_plan_e.K.Free=zeros(4,3);
plan_e.Structure=S_plan_e;
out_f.StateName={'V','alpha','pitch','wz'};
out_f.StateUnit={'m/s','бу','бу','бу'};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x0=[V_f(1) pitch_f(1) pitch_f(1) wz_f(1)];
% opt = ssestOptions;
% opt.InitialState = idpar(x0);
% opt.InitialState.Free(2) = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in=[delta_T_f delta_e_f];
out=[V_f1 pitch_f wz_f ];
T=0.02;
data_f=iddata(out,in,T);
data_f.InputName={'delta_T','delta_e'};
data_f.OutputName={'V','pitch','wz'};
data_f.InputUnit={'бу','бу'};
data_f.OutputUnit={'m/s','бу','бу/s'};
opt = ssestOptions;
opt.InitializeMethod='auto';
opt.SearchMethod='auto';
%'auto' (default) | 'gn' | 'gna' | 'lm' | 'grad' | 'lsqnonlin' | 'fmincon'
opt.SearchOptions.MaxIterations = 50;
opt.Display='on';
opt.InitialState=x0;  
opt.Focus='prediction';
%  opt.InitialState='backcast';
% opt.EstimateCovariance=0; 
% opt.OutputWeight='noise';
% opt.Regularization.Lambda=100;

out_f=ssest(data_f,plan_e,opt );

compare(data_f,out_f)
