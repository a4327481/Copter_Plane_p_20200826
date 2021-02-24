% in=loadjson('param_driverg.json');
% fid=fopen('data_devg2.m','w');
% len=length(in.ParamDriver  );
% for i=1:len
%  temp=[in.ParamDriver{1, i}.nameFull,'=',num2str(in.ParamDriver{1, i}.value),';%%',num2str(in.ParamDriver{1, i}.name),'\n'];    
% 
%  fprintf(fid,temp);
% end
%  fclose(fid);  
in=loadjson('ParamDriver.json');
in_Full=loadjson('ParamDriver_Full.json');
fid=fopen('data_devg2.m','w');
len=length(in.ParamDriver  );
for i=1:len
 temp=[in_Full.ParamDriver{1, i}.nameFull,'=',num2str(in.ParamDriver{1, i}.v),';%%',num2str(in.ParamDriver{1, i}.n),'\n'];    

 fprintf(fid,temp);
end
 fclose(fid); 