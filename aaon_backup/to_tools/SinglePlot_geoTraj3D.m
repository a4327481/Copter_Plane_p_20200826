% 绘制地图信息
uif = uifigure;
g = geoglobe(uif);
% lat_geo = sensors.Algo_sl.algo_NAV_lat;
% lon_geo = sensors.Algo_sl.algo_NAV_lon;
% alt_geo = sensors.Algo_sl.algo_NAV_alt;
try
    lat_geo = sensors.GPS.ublox_lat;
    lon_geo = sensors.GPS.ublox_lon;
    alt_geo = sensors.GPS.ublox_height;
catch
    disp('载入demo数据')
    load demo
end
idxNoZero = lat_geo==0;
lat_geo(idxNoZero) = [];
lon_geo(idxNoZero) = [];
alt_geo(idxNoZero) = [];
geoplot3(g,lat_geo,lon_geo,alt_geo,'y','HeightReference','geoid', ...
    'LineWidth',3)

uif.WindowButtonDownFcn = @callback_WindowButtonDownFcn;



function callback_WindowButtonDownFcn(~,~)
disp('1')
end