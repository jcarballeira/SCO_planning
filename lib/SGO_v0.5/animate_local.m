
function animate_local(robot, qq)
global parameters

for i=1:1:size(qq,2)
     drawrobot3d(robot, qq(:,i))    
     for j=1:size(parameters.trajectory,2)
         line_work = parameters.trajectory{j}.line;
         plot3(line_work(1,:),line_work(2,:),line_work(3,:))
     end
     %plot obstacles
     for j=1:size(parameters.obstacles,2)
%          points = parameters.obstacles{j}.points;
%          v = [points(:,1)'; 
%               points(:,2)';
%               points(:,3)'];
%          f = [1 2 3];
%          patch('Faces',f,'Vertices',v,'FaceColor','blue')
        
        sph_ctr= parameters.obstacles{j}.center;
        sph_radio=parameters.obstacles{j}.radio;
         % Define the input grid (in 3D)
        [x3, y3, z3] = meshgrid(linspace(-1,1));
        % Compute the implicitly defined function (sphere)
        f1 = (x3-sph_ctr(1)).^2 + (y3-sph_ctr(2)).^2 + (z3-sph_ctr(3)).^2 - sph_radio;

        % Next surface is z = 2*y - 6*x^3, which can also be expressed as
        % Visualize the two surfaces.
        patch(isosurface(x3, y3, z3, f1, 0), 'FaceColor', [0.5 1.0 0.5], 'EdgeColor', 'none');
        %patch(isosurface(x3, y3, z3, f2, 0), 'FaceColor', [1.0 0.5 0.0], 'EdgeColor', 'none');
        view(3); camlight; axis vis3d;
     end
end


