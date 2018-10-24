
function pathG = build_initial_path(first_pos)
global parameters
global robot
pathG=[];
first_pos=1;
    line = parameters.trajectory{1}.line;
%     p0 = line(:,1);
%     p1 = line(:,2);
    T0 = parameters.trajectory{1}.T0;
    for p=1:parameters.N
        pnt=line(:,first_pos+p-1);
        pathG{p}.T=obtain_TFmatrix(pnt);
    end    
    
    %partial_path = build_partial_path(p0, p1, T);
    %append
%     for k=1:size(partial_path,2)
%         pathG{j}.T=partial_path{k}.T;
%         j=j+1;
%     end

end

function T=obtain_TFmatrix(pt)
global parameters
global robot
n=pt-parameters.obstacles{1}.center;
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
T(1:3,4)=pt;
T(4,4)=1;

%constant orientation
%T = eye(4);

%return T in the oposite direction!
%with robot coupling tranformation
T=T*inv(robot.Tcoupling);
end


function partial_path = build_partial_path(p0, pf, T0)
global parameters
%this defines the line to follow in direction
deltaV = pf - p0;
deltaV = deltaV/norm(deltaV);
%and total length of the movement
total_length = norm(pf - p0); %m
%linspace!!
mov = linspace(0, total_length, parameters.N);
%line0 = [p0 pf];
%plot(line0(1,:),line0(2,:),'k')

partial_path=[];
Ti = T0;
for i=1:length(mov)    
    fprintf('Fast global plan %d out of %d\n', i, length(mov))
    %update to next point in trajectory
    Ti(1:3,4) = p0 + mov(i)*deltaV;   
    partial_path{i}.T = Ti;
end
end