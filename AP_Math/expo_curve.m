function  output=expo_curve( alpha,  x)
% /* cubic "expo" curve generator
%  * alpha range: [0,1] min to max expo
%  * input range: [-1,1]
%  */
    output= (1.0 - alpha) * x + alpha * x * x * x;
end

