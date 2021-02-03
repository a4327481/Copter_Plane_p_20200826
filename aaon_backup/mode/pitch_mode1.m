
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
A=[ a22 a23 a24; 0 0 1; a42 0 a44];
B = [ b22; 0; b42];
% A=out_f.A;
% B=out_f.B;
C = [ 0 1 0]; 
% C = [1 0 0 0;0 0 1 0]; 
D = 0; 
K = zeros(3,1);

Ve=17;
% V_f1=V_f-Ve;
% x0=[V_f1(1) pitch_f(1) pitch_f(1) wz_f(1)]';
x0=[  pitch_f(1) pitch_f(1) wz_f(1)]';
Ts = 0;

plan_e = idss(A,B,C,D,K,x0,Ts);
S_plan_e = plan_e.Structure; 
S_plan_e.A.Free=[ 1 1 1;  0 0 0;  1 0 1];
S_plan_e.B.Free=[ 1  ;0  ;1  ];
S_plan_e.C.Free=zeros(1,3);
S_plan_e.D.Free=zeros(1,1);
S_plan_e.K.Free=zeros(3,1);
plan_e.Structure=S_plan_e;
% out_f.StateName={'V','alpha','pitch','wz'};
% out_f.StateUnit={'m/s','бу','бу','бу'};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x0=[V_f(1) pitch_f(1) pitch_f(1) wz_f(1)];
% opt = ssestOptions;
% opt.InitialState = idpar(x0);
% opt.InitialState.Free(2) = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in=[delta_e_f];
out=[pitch_f ];


T=0.02;
data_f=iddata(out,in,T);
% data_f.InputName={'delta_e'};
% data_f.OutputName={'pitch','wz'};
% data_f.InputUnit={'бу','бу'};
% data_f.OutputUnit={'m/s','бу','бу/s'};
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
% hold on
% plot(ts1-850,pitch_re_f,'bla')