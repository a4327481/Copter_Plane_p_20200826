function output = NotchFilter_apply(sample,ret)

persistent ntchsig2 ntchsig1 ntchsig signal2 signal1
if isempty(ntchsig2)
    ntchsig2 = 0;
end
if isempty(ntchsig1)
    ntchsig1 = 0;
end
if isempty(ntchsig)
    ntchsig = 0;
end
if isempty(signal2)
    signal2 = 0;
end
if isempty(signal1)
    signal1 = 0;
end
   b0 = ret.b0;
   b1 = ret.b1;
   b2 = ret.b2;
   a1 = ret.a1;
   a2 = ret.a2;
   a0_inv =ret.a0_inv;
    

if (~ret.init)
    % if we have not been initialised when return the input
    % sample as output and update delayed samples
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    signal2 = signal1;
    signal1 = sample;
    output= sample;
else
    
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    output = (ntchsig*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2) * a0_inv;
    signal2 = signal1;
    signal1 = output;
end
end

