%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulate a spring mass problem using a RK4 method.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function spring_mass()
global m k c f
close all;

%initial and final time
t0 = 0;
tfinal = 20;
%initial position and speed
x0 = 0;
xd0 = 0;
% mass, force and constants
f = 13; %N
c = 15; % N/m/s
m = 2; % kg
k = 15; %N/m


[t, x] = runge_kutta(@spring_mass_equation, [x0 xd0]', [t0 tfinal], 0.01);

%plot results
figure, 
plot(t, x(1,:), 'r'), hold
plot(t, x(2,:), 'b')
legend('Position (m)', 'Speed (m/s)')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper function to solve a second order differential equation.
% Called from function exerciseB()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xd = spring_mass_equation(t, x)
global m k c f

c = 15+0.8*t;
w=15;
f = sin(w*t);

xd1 = x(2);
xd2 = (1/m)*(f-c*x(2)-k*x(1));

xd = [xd1 xd2]';


