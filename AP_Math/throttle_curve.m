function thr_out=throttle_curve(  thr_mid,   alpha,   thr_in)
% /* throttle curve generator
%  * thr_mid: output at mid stick
%  * alpha: expo coefficient
%  * thr_in: [0-1]
%  */
alpha2 = alpha + 1.25 * (1.0 - alpha) * (0.5 - thr_mid) / 0.5;
alpha2 = constrain_value (alpha2, 0.0, 1.0);
thr_out = 0.0;
if (thr_in < 0.5)
    t = linear_interpolate(-1.0, 0.0, thr_in, 0.0, 0.5);
    thr_out = linear_interpolate(0.0, thr_mid, expo_curve(alpha, t), -1.0, 0.0);
else
    t = linear_interpolate(0.0, 1.0, thr_in, 0.5, 1.0);
    thr_out = linear_interpolate(thr_mid, 1.0, expo_curve(alpha2, t), 0.0, 1.0);
end
end

