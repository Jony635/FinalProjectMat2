function [axis,angle] = rotMat2Eaa(rot_matrix)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    angle=acos(((trace(rot_matrix))-1)/2);
    m_axis=(rot_matrix-rot_matrix')/(2*sin(angle));
    axis=[m_axis(3,2);m_axis(1,3);m_axis(2,1)];
end

