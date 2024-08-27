%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
geom_model = BuildTree();
disp('Geometric Model')
disp(geom_model);
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = [0; 0; 0; 0; 1; 0; 0;];  % specify the link type for all joints: Rotational = 0, Prismatic = 1.

%% Given the following configuration compute the Direct Geometry for the manipulator
q = [0, 0, 0, 0, 0, 0, 0]';

% Compute biTei : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
biTei = GetDirectGeometry(q, geom_model, linkType);
disp('biTei')
disp(biTei);

% Compute the transformation of the ee w.r.t. the robot base
bTe = GetTransformationWrtBase(biTei, 7);
disp('bTe')
disp(bTe)

%% Given the previous joint configuration compute the Jacobian matrix of the manipulator
J = GetJacobian(biTei, bTe, linkType);
disp('Jacobian Matrix for the given q')
disp(J)

% Tool frame definition
eTt = eye(4);
eRt = [1 0 0;
    0 0 -1;
    0 1 0;];
eOt = [0; 0; 0.25;]; 
eTt(1:3,1:3) = eRt;
eTt(1:3,4) = eOt;
bTt = bTe * eTt;

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);


%% Inverse Kinematic

% Simulation variables
% simulation time definition 
ts = 0.001;
t_start = 0.0;
t_end = 10.0;
t = t_start:ts:t_end; 

% control proportional gain 
angular_gain = 0.5;
linear_gain = 0.5;

% preallocation variables
bTi = zeros(4, 4, numberOfLinks);
bri = zeros(3, numberOfLinks);
x_dot = zeros(6,1);
error_linear = zeros(3,1);
error_angular = zeros(3,1); 

% joints upper and lower bounds
qmin = -3.14 * ones(8,1);
qmin(5) = 0;
qmax = +3.14 * ones(8,1);
qmax(5) = 1;

% initial configuration  
q = [0, pi/3, 0, pi/4, 0, -pi/6, 0]';

% recompute the bTe in this configuration
biTei = GetDirectGeometry(q, geom_model, linkType);
bTe = GetTransformationWrtBase(biTei, numberOfLinks);
bTt = bTe * eTt;
disp('bTt q = initial config');
disp(bTt);

% Goal definition 
theta = (pi/6)
bRg = [cos(theta) -sin(theta) 0;
        sin(theta) cos(theta) 0;
        0 0 1;] 
bOt = bTt(1:3,4);
tOg = [-0.3 ; -0.3 ; -0.25;];
disp (bOt);
disp (tOg);
bOg = bOt + bTt(1:3,1:3) * tOg;
bTg = eye(4);
bTg(1:3,1:3) = bRg;
bTg(1:3,4) = bOg;    
bVg = [-0.01; 0.02; 0;];
disp('bTg')
disp(bTg)
   
% Show simulation ? %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_simulation = true;
tool = true;

% visualization, do not change
figure
grid on 
hold on
title('MOTION OF THE MANIPULATOR')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
csize = length(t);
cmap = colormap(parula(csize));
color = cmap(mod(cindex,csize)+1,:);
plot3(bOg(1),bOg(2),bOg(3),'ro')


%%%%%%% Kinematic Simulation %%%%%%%
x_dot_hist = [];
t_hist = [];
err_lin_hist = [];
for i = t
    
    % update goal position
    bTg(1:3,4) = bTg(1:3,4) + bVg*ts;

    % computing transformation matrices for the new configuration 
    biTei = GetDirectGeometry(q, geom_model, linkType);
    % computing transformation matrix from base to the tool
    bTe = GetTransformationWrtBase(biTei, numberOfLinks);
    bTt = bTe * eTt;
    % computing the tool jacobian 
    bJe = GetJacobian(biTei, bTe, linkType);
    eWt = bTt(1:3,4) - bTe(1:3,4);
    x = eWt(1);
    y = eWt(2);
    z = eWt(3);
    rx = [0 -z y;
        z 0 -x;
        -y x 0;];
    S = eye(6);
    S(4:6,1:3) = -rx;
    bJt = S * bJe;
    
    % Compute the cartesian error to reach the goal
    A = eye(6);
    A(1:3,1:3) = angular_gain * A(1:3,1:3);
    A(4:6,4:6)  = linear_gain * A(4:6,4:6);
    tRg = pinv(bTt(1:3,1:3))*bTg(1:3,1:3);
    [theta, v] = ComputeInverseAngleAxis(tRg);
    t_pho = theta*v;
    b_pho = bTe(1:3,1:3) * eRt * t_pho;
    
    error_angular = b_pho;

    error_linear = bTg(1:3,4) - bTt(1:3,4);

    err = zeros(6,1);
    err(1:3,1) = error_angular;
    err(4:6,1) = error_linear;


    %% Compute the reference velocities
    x_dot = A * err + [0; 0; 0; -0.01; 0.02; 0];
    %x_dot(1:3) = ...;
    %x_dot(4:6) = ...;;
    %% Compute desired joint velocities 
    q_dot = pinv(bJt) * x_dot 
        

    % computing the actual velocity and saving the unitary direction for plot
    % do NOT change
    x_dot_actual = bJt*q_dot;
    x_dot_hist = [x_dot_hist; (x_dot_actual/norm(x_dot_actual))'];
    t_hist = [t_hist; i];
    err_lin_hist = [err_lin_hist; norm(error_linear)];

    % simulating the robot
    q = KinematicSimulation(q, q_dot, ts, qmin, qmax);

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        
        for n =1:numberOfLinks
            bTi(:,:,n)= GetTransformationWrtBase(biTei,n);
        end
    
        bri(:,1) = [0; 0; 0];
    
        for j = 1:numberOfLinks
            bri(:,j+1) = bTi(1:3,4,j);
            if tool == true
                brt = bTt(1:3,4);
                bri(:,j+2) = brt;
            end
    
        end
    
        for j = 1:numberOfLinks+1
            plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
            plot3(bTg(1,4),bTg(2,4),bTg(3,4),'ro')
            if tool == true
                plot3(brt(1),brt(2),brt(3),'go')
            end
        end
    
        color = cmap(mod(cindex,csize)+1,:);
        cindex = cindex + 1;
    
        line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)
    end


    if show_simulation == true
        drawnow
        
    end    
    if(norm(error_angular) < 0.01 && norm(error_linear) < 0.001)
        disp('REACHED REQUESTED POSE')
        svd(bJt)
        
        break
    end

end

%%Plot the final configuration of the robot
figure
grid on 
hold on
title('FINAL CONFIGURATION')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
for j = 1:numberOfLinks+1
    plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
    if tool == true
        plot3(brt(1),brt(2),brt(3),'go')
    end
end

color = cmap(mod(cindex,csize)+1,:);
cindex = cindex + 1;

line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)

figure;
title('Distance to Goal')
plot(t_hist, err_lin_hist,'LineWidth', 1.5)
legend('|error linear|')
xlabel('Time (s)')
ylabel('Distance (m)')

