function [ eul ] = quat2eul( quat )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q_0 = quat(1);
q_1 = quat(2);
q_2 = quat(3);
q_3 = quat(4);
eul(3,1) = atan2(2*(q_1*q_2 + q_0*q_3), q_0*q_0 + q_1*q_1 - q_2*q_2 - q_3*q_3);
eul(2,1) = asin(-2*(q_1*q_3 - q_0*q_2));
eul(1,1) = atan2(2*(q_2*q_3 + q_0*q_1), q_0*q_0 - q_1*q_1 - q_2*q_2 + q_3*q_3);

end

