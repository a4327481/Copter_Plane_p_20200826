function loc_run()
global Curr_sate
global Copter_Plane
loc                   = Copter_Plane.loc;

loc.num=Curr_sate.wp_in(:,1);
loc.lat=Curr_sate.wp_in(:,2);
loc.lon=Curr_sate.wp_in(:,3);
i_lat=find(loc.lat==0,1);
i_lon=find(loc.lon==0,1);
i_num=max(i_lat,i_lon);
if(i_num<2)
    i_num=2;
end
if(all((loc.lat(i_num-1)-loc.lat(2))>1e-6)||all((loc.lon(i_num-1)-loc.lon(2))>1e-6))
    loc.lat(i_num)=loc.lat(2);
    loc.lon(i_num)=loc.lon(2);  
end
Copter_Plane.loc                   =loc;

 end

