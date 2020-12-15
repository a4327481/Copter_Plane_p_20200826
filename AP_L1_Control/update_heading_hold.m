function update_heading_hold( navigation_heading_cd)
global L1_period
global target_bearing_cd
global nav_bearing
global yaw
global groundspeed_vector
global L1_dist
global bearing_error
global latAccDem
global HD
% update L1 control for heading hold navigation

    % Calculate normalised frequency for tracking loop
      omegaA = 4.4428/L1_period; % sqrt(2)*pi/period
    % Calculate additional damping gain

    % copy to _target_bearing_cd and _nav_bearing
    target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    nav_bearing = radians(navigation_heading_cd * 0.01);

    Nu_cd = target_bearing_cd - wrap_180_cd(yaw*100*HD);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01);


    %Calculate groundspeed
     groundSpeed = norm(groundspeed_vector,2);

    % Calculate time varying control parameters
    L1_dist = groundSpeed / omegaA; % L1 distance is adjusted to maintain a constant tracking loop frequency
    VomegaA = groundSpeed * omegaA;

    % Waypoint capture status is always false during heading hold
%     _WPcircle = false;

%     _crosstrack_error = 0;

    bearing_error = Nu; % bearing error angle (radians), +ve to left of track

    % Limit Nu to +-pi
    Nu = constrain_value(Nu, -pi/2, pi/2);
    latAccDem = 2.0*sin(Nu)*VomegaA;

%     _data_is_stale = false; % status are correctly updated with current waypoint data
end
