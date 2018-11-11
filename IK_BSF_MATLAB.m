%Inverse using MATLAB
t = (0:0.2:10)'; % Time
count = length(t);
center = [15.0 5.0 0];
radius = 10;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(robot);
ndof = length(q0);
%qs = zeros(count, ndof);


ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess
clear configSol ;
[configSol1,solInfo1] = ik(endEffector,trvec2tform(points(1,:)),weights,qInitial);
qs = struct(configSol1);
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    [configSol,solInfo] = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = configSol;
    % Start from prior solution
    qInitial = configSol;
end

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end