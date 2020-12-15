a=1:500;
len=length(a);
pos_in=[a'  a'  a'.*0];
out=pos_in;
for i=1:len
out(i,:)=sqrt_controller_pos(pos_in(i,:), POSCONTROL_POS_XY_P, POSCONTROL_ACCEL_XY);
end
plot(out(:,1))