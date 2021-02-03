% in=[delta_e_f];
% out=[pitch_f wz_f ];
% len=length(pitch_f);
% pitch_f_m=zeros(len,1);
% pitch_f_m(1)=pitch_f(1);
% for i=2:len
%     pitch_f_m(i)=pitch_f_m(i-1)+wz(i)*0.02;
% end
% plot(ts1,pitch_f,ts1,pitch_f_m,'bla')
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data_ck=[data.ARSP(:,2)/1000,data.ARSP(:,3:4),data.ARSP(:,5)/100,data.ARSP(:,6:7)];
fid=fopen([PathName,'\\','ARSP.dat'],'w');
fprintf(fid,'t Airspeedø’ÀŸ DiffPress—π≤Ó TempŒ¬∂» RawPress—π≤Ó‘≠ ˝ Offset∆´÷√\n');
ts=data.ARSP(:,2)/1000;
V_ARSP=data.ARSP(:,3);

hold on
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
V_ARSP_f= filter(b,a,V_ARSP);
plot(ts,V_ARSP_f)
windowSize = 50; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
V_ARSP_f= filter(b,a,V_ARSP);
plot(ts,V_ARSP_f)
windowSize = 500; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
V_ARSP_f= filter(b,a,V_ARSP);
plot(ts,V_ARSP_f)