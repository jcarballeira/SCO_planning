
% Compute null space vanilla
function [qd_null] = null_space_4dof(robot, q)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
% as it is a planar robot it is aonly able of moving on vx, vy and wz!!!
J = [J(1:2,:); J(6,:)]
I = eye(robot.DOF);
Jp=pinv(J);
%null space projector
%n_space_projector = (I-Jp*J);
%for an arbitrary vector
%do not use [1 1 1 1]
qd1 = [1 0 0 0]';
qd2 = [0 1 0 0]';
qd3 = [0 0 1 0]';
qd4 = [0 0 0 1]';
qd_null1 = project(J, Jp, I, qd1); 
qd_null2 = project(J, Jp, I, qd2);
qd_null3 = project(J, Jp, I, qd3);
qd_null4 = project(J, Jp, I, qd4);

qd_null_v = [qd_null1 qd_null2 qd_null3 qd_null4];

qd_null_norm = [norm(qd_null1) norm(qd_null2) norm(qd_null3) norm(qd_null4)];
%return the max norm projection, the wider velocity projection
[y, index] = max(qd_null_norm);
qd_null = qd_null_v(:,index);

function qd_null = project(J, Jp, I, qd)
%q2 est� calculado a trav�s de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
qd_null = (I-Jp*J)*qd;

 
