function result = se3_log(T) %T, 4x4
%%
%%
R=T(1:3,1:3);
t=T(1:3,4);
phi=so3_log(R);
J=jaco_left(phi);
p=J\t;
result=[phi;p];
end

