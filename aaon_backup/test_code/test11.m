len=length(x);
b=zeros(len,1);
for i=1:len
b(i)=deadzonef(x(i),0.05,1);
end
plot(x,b)