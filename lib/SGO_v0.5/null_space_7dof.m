% Compute null space vanilla
function [qd_null] = null_space_7dof(robot, q)
%compute nil-space!!
J = manipulator_jacobian(robot, q);
I = eye(robot.DOF);
Jp=pinv(J);
%null space projector
%n_space_projector = (I-Jp*J);
%for an arbitrary vector
%do not use [1 1 1 1]
qd1 = [1 0 0 0 0 0 0]';
qd2 = [0 1 0 0 0 0 0]';
qd3 = [0 0 1 0 0 0 0]';
qd4 = [0 0 0 1 0 0 0]';
qd5 = [0 0 0 0 1 0 0]';
qd6 = [0 0 0 0 0 1 0]';
qd7 = [0 0 0 0 0 0 1]';
qd_null1 = project(J, Jp, I, qd1);
qd_null2 = project(J, Jp, I, qd2);
qd_null3 = project(J, Jp, I, qd3);
qd_null4 = project(J, Jp, I, qd4);
qd_null5 = project(J, Jp, I, qd5);
qd_null6 = project(J, Jp, I, qd6);
qd_null7 = project(J, Jp, I, qd7);


qd_null_v = [qd_null1 qd_null2 qd_null3 qd_null4 qd_null5 qd_null6 qd_null7];

qd_null_norm = [norm(qd_null1) norm(qd_null2) norm(qd_null3) norm(qd_null4) norm(qd_null5) norm(qd_null6) norm(qd_null7)];
%return the max norm
[y, index] = max(qd_null_norm);
qd_null = qd_null_v(:,index);

function qd_null = project(J, Jp, I, qd)
%q2 est� calculado a trav�s de un proyector (I-Jp*J),
%, de tal manera que q2 pertenece al null space de J
qd_null = (I-Jp*J)*qd;

 
