out_temp=[out.tout,out.NavFilterRes.posNED.data ,out.NavFilterRes.velNED.data,out.NavFilterRes.pitchd.data,out.NavFilterRes.rolld.data,out.NavFilterRes.yawd.data];

head=['t PN PE PD VN VE VD pitchd rolld yawd\n'];
data_ck=out_temp;
fid=fopen([subFoldName,'·ÂÕæÊý¾Ý_',temp,'.bin'],'w');
fprintf(fid,head);
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid); 
