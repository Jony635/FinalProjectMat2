function [ r_euler_matrix ] = eAngles2rotM( phi,theta,psi )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
r_euler_matrix=[cos(theta)*cos(psi) cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi) cos(psi)*cos(phi)*sin(theta)+sin(phi)*sin(phi);
                cos(theta)*sin(psi) sin(psi)*sin(theta)*sin(phi)+cos(phi)*cos(psi) sin(psi)*sin(theta)*cos(phi)-cos(phi)*sin(phi);
                -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)]

end

