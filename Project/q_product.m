function [ q_c ] = q_product( q_a,q_b )
%Q_PRODUCT Summary of this function goes here
%   Detailed explanation goes here
q_c=zeros(4,1);

q_a_0=q_a(1);
q_a_v=[q_a(2);q_a(3);q_a(4)];

q_b_0=q_b(1);
q_b_v=[q_b(2);q_b(3);q_b(4)];

q_c(1)=q_a_0*q_b_0-(q_a_v'*q_b_v);
q_c(2:4)=q_a_0*q_b_v+q_b_0*q_a_v+cross(q_a_v,q_b_v);

end

