%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inverse kinematics for the 3dof planar robot
% T: homogeneous matrix
% robot: structure with arm parameters
% returns: all possible solutions or q = [q1 q2] that place the end effectors at the
% position specified by T. Two possible solutions q1 and q2 q3 and q4 are returned,
% generally called elbow up and elbow down, combined with 
%   Author: Arturo Gil Aparicio arturo.gil@umh.es
%   Date: 08/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function q = inversekinematic_4dofplanarb(robot,  Tf, q0)

fprintf('\nComputing inverse kinematics for the %s robot', robot.name);
fprintf('\nUsing a Moore-penrose pseudo-inverse based method');

%la matriz de posición/orientación en Quaternion representation
Qf = T2quaternion(Tf);
Pf = Tf(1:3,4);

q=q0;
step_time = 0.05;
%step 1, inverse kinematics this is a gradient descent solution based on moore-penrose inverse
while 1 
    Ti=directkinematic(robot, q);
    Qi = T2quaternion(Ti);
    Pi = Ti(1:3,4);
   
    eps1= reached_position(Pf, Pi);
    eps2= reached_orientation(Qf, Qi);
    %compute linear speed and angular speed that are served as a high level
    %based on the current pose
    v0 = compute_high_level_action_kinematic_v(Pf, Pi); %1m/s 
    w0 = compute_high_level_action_kinematic_w(Qf, Qi); %1rad/s
   
    %the restriction is the speed to reach the point
    %Caution!, given that it is a planar robot, specify only the last wz of
    %rotation as Vref
    Vref = [v0' w0']';
    if eps1 < 0.001 && eps2 < 0.000001
        break;
    end
    
    % Different strategies can be used here
    % moore_penrose finds the minimum norm(qd) involved
    qd = inverse_kinematic_moore_penrose(robot, q, Vref);
   
    %actually move the robot.
    q = q + qd*step_time;
end




function qd = inverse_kinematic_moore_penrose(robot, q, Vref)
J = manipulator_jacobian(robot, q);
Jp = pinv(J);
qd = Jp*Vref;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [v] = compute_high_level_action_kinematic_v(Pf, Pi)
%compute a constant linear speed till target
v = (Pf-Pi);
v = v(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%En base a la posición y orientación final, calcular cuáles deben ser las
%velocidades...
% Esto es diferente a calcular la velocidades cuando ya hay contacto y se
% trata de un problema de control... pero es parecido
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [w] = compute_high_level_action_kinematic_w(Qf, Qi)
%compute a constant angular speed till target
%asume the movement is performed in 1 second
w = angular_w_between_quaternions(Qi, Qf, 1);
w = w(:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute angular speed w that moves Q0 into Q1 in time total_time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function w = angular_w_between_quaternions(Q0, Q1, total_time)
%Let's first find quaternion q so q*q0=q1 it is q=q1/q0 
%For unit length quaternions, you can use q=q1*Conj(q0)
Q = qprod(Q1, qconj(Q0));

%To find rotation velocity that turns by q during time Dt you need to 
%convert quaternion to axis angle using something like this:
len=sqrt(Q(2)^2 + Q(3)^2 + Q(4)^2);
angle=2*atan2(len, Q(1));
%vector3 axis;
if len > 0
    axis=[Q(2) Q(3) Q(4)]./len;
else
    axis=[1 0 0];
end
w=axis*angle/total_time;


% function animate_local(robot, q)
% global configuration 
% 
% h=figure(configuration.figure.robot);, hold on,
% 
% %get adjusted view
% [az,el] = view;
% for j=1:size(q, 2),
%     clf(h);
%     qj=q(:,j);  
%     view(az,el);
%    
%     drawrobot3d(robot, qj);  
%     
%     %pause to get a nice view
%     pause(0.1);   
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_orientation(Qf, Qi)
Q = Qf-Qi;
reach = sqrt(Q(1)^2 + Q(2)^2 + Q(3)^2 + Q(4)^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check whether orientation has been reached
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reach = reached_position(Pf, Pi)
P = Pf-Pi;
reach = sqrt(P(1)^2 + P(2)^2 + P(3)^2);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % normalize vector if possible.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function v = normalize(v)
% d = sqrt(v(1)^2+v(2)^2+v(3)^2);
% if d>0
%     v = v/d;
% end
% v=v(:);
% 
% function k = saturate(k, min_value, max_value)
% if k < min_value
%     k = min_value;
% end
% if k > max_value
%     k = max_value;
% end