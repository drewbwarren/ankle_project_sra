function [out] = quat2angle(q)

q = q/norm(q);
w = q(1);
x = q(2);
y = q(3);
z = q(4);

sinr = 2*(w*x + y*z);
cosr = 1 - 2*(x*x + y*y);
roll = atan2(sinr,cosr);

sinp = 2*(w*y - z*x);
if sinp > 1.0
    sinp = 1.0;
elseif sinp < -1.0
    sinp = -1.0;
end
pitch = asin(sinp);


siny = 2*(w*z + x*y);
cosy = 1 - 2*(y*y + z*z);
yaw = atan2(siny,cosy);


out = radtodeg([roll,pitch,yaw]);



end

