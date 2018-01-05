function [matrix] = Eaa2rotMat(axis,angle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes her
    %if sqrt(axis(3,1)^2+axis(2,1)^2+axis(1,1)^2)==1
    axis = axis/norm(axis);
    
    I=eye(3);

        u=[0 -axis(3,1) axis(2,1);axis(3,1) 0 -axis(1,1);-axis(2,1) axis(1,1) 0];

        matrix=I*cos(angle)+(1-cos(angle))*(axis*axis')+u*sin(angle);
    %end
end

