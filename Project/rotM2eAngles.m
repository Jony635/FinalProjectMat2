function [ phi,theta,psi ] = rotM2eAngles( euler_matrix )
%ROTM2EANGLES Summary of this function goes here
%   Detailed explanation goes here
if(euler_matrix(3,1)== 1)
    phi=0;
    theta=1;
    psi=acos(euler_matrix(2,2));
else if(euler_matrix(3,1)== -1)
        phi=0;
        theta=-1;
        psi=-acos(euler_matrix(2,2)); 
    else
theta=-asin(euler_matrix(3,1));
phi=atan2(euler_matrix(3,2)/cos(theta),euler_matrix(3,3)/cos(theta));
psi=atan2(euler_matrix(2,1)/cos(theta),euler_matrix(1,1)/cos(theta));
    end
end

