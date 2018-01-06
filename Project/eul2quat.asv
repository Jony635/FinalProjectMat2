function [ quat ] = eul2quat( phi,theta,psi)
quat = zeros(4,1);

c_1 = cos(psi*0.5);
s_1 = sin(psi*0.5);
c_2 = cos(theta*0.5);
s_2 = sin(theta*0.5);
c_3 = cos(phi*0.5);
s_3 = sin(phi*0.5);

quat(1,1) = c_1*c_2*c_3 + s_1*s_2*s_3;
quat(2,1) = c_1*c_2*s_3 - s_1*s_2*c_3;
quat(3,1) = c_1*s_2*c_3 + s_1*c_2*s_3;
quat(4,1) = s_1*c_2*c_3 - c_1*s_2*s_3;
end

