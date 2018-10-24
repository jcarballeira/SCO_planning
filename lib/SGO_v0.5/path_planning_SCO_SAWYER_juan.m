%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max manipulability index ALONG A LINE.
% Use stomp like to optimize along a surface/line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [qq, manips]=path_planning_SCO_SAWYER_juan
close all;
global robot
global parameters
global hfigures

%STOMP PARAMETERS
%conversion from cost to Prob factor
%parameters.lambda = .4;
parameters.lambda_obstacles = 2;
parameters.lambda_manip = 3;
%cost function starts at this distance
%must be below 0.3 for the 4 DOF robot
parameters.epsilon_distance = 0.1;
%height of the obstacle
%parameters.yo = 2.5;

%parameters.noise_sigma_null_space = 0.01;
parameters.alpha=0.05;
parameters.step_time=0.1;
%Error in XYZ to stop inverse kinematics
parameters.epsilonXYZ=0.01;
%Error in Quaternion to stop inverse kinematics.
parameters.epsilonQ=0.01;
parameters.stop_iterations=500;

%number of waypoints
parameters.N = 25;
%number of particles
parameters.K = 15;
%select the rows of J that should be taken into account when computing
%manipulability
parameters.sel_J = [1 2 3 4 5 6];
%parameters.sel_J = [1 2 3];
parameters.obstacles = [];

parameters.animate = 0;
close all
% hfigures.hpaths = figure;
% hfigures.hcosts = figure;
% hfigures.hee = figure;
% hfigures.htheta = figure;
% hfigures.hbest_costs = figure;
% hfigures.hdtheta = figure;

sph_ctr=[0 0.7 1]';
sph_radio=0.5^2;
% Define the input grid (in 3D)
[x3, y3, z3] = meshgrid(linspace(-1,1));
% Compute the implicitly defined function (sphere)
f1 = (x3-sph_ctr(1)).^2 + (y3-sph_ctr(2)).^2 + (z3-sph_ctr(3)).^2 - sph_radio;

% Next surface is z = 2*y - 6*x^3, which can also be expressed as
% 2*y - 6*x^3 - z = 0.
f2 = 2*y3 - 6*x3.^3 - z3;
% Also compute z = 2*y - 6*x^3 in the 'traditional' way.
[x2, y2] = meshgrid(linspace(-1,1));
z2 = 2*y2 - 6*x2.^3;
% Visualize the two surfaces.
patch(isosurface(x3, y3, z3, f1, 0), 'FaceColor', [1 1 1], 'EdgeColor', 'none');
%patch(isosurface(x3, y3, z3, f2, 0), 'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none');
%view(3); camlight; axis vis3d;

% Find the difference field.
f3 = f1 - f2;
% Interpolate the difference field on the explicitly defined surface.
f3s = interp3(x3, y3, z3, f3, x2, y2, z2);
% Find the contour where the difference (on the surface) is zero.
C = contours(x2, y2, f3s, [0 0]);
% Extract the x- and y-locations from the contour matrix C.
xL = C(1, 2:end);
yL = C(2, 2:end);
% Interpolate on the first surface to find z-locations for the intersection
% line.
zL = interp2(x2, y2, z2, xL, yL);
tr=[xL ;yL; zL];
% Visualize the line.
%line(xL,yL,zL,'Color','k','LineWidth',3);


parameters.obstacles{1}.center = sph_ctr;
parameters.obstacles{1}.radio = sph_radio;

%define trajectory over sphere
parameters.trajectory{1}.line = tr;
inicio_traj=ceil(uniform(0, size(tr,2)-parameters.N-1, 1, 1));
p0=tr(:,inicio_traj);
parameters.trajectory{1}.p0=p0;
parameters.trajectory{1}.first_pos=inicio_traj;
parameters.trajectory{1}.T0 = build_T_from_obstacle(tr,p0);

%LAUNCH SCO given the stored parameters
pk = SCO_null_space(robot);


% Given the normal, we must compute an orientation that is perpendicular
% to the normal
% surface sign indicates whether the Z vector of the end_effector must have
% the same direction as n or opposite.
function T = build_T_from_obstacle(tr,p0)
global robot
global parameters

n=tr(:,1)-parameters.obstacles{1}.center;

%this is vector z7 of the end effector
z7 = n;
x0=[1 0 0]';
%x7 points in the direction of the first line
x7 = cross(x0, n);

%y7 to form rotation matrix
y7 = cross(z7,x7);

T = zeros(4,4);
R = [x7 y7 z7];
T(1:3,1:3)=R;
T(1:3,4)=p0;
T(4,4)=1;

%constant orientation
%T = eye(4);

%return T in the oposite direction!
%with robot coupling tranformation
T=T*inv(robot.Tcoupling);



 
