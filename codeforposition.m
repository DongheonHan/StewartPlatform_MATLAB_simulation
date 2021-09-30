%{
Author: Dongheon Han 
Forward kinematics of Stewart Platform
LIDAR Athena's Neck Team
%}
close, clear, clc;
%% Input
psi = deg2rad(input('Yaw input   (degree): '));
theta = deg2rad(input('Pitch input (degree): '));
phi = deg2rad(input('Roll input  (degree): '));
%% Rotation Matrix
rotationMatrix = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);...
    sin(psi)*cos(theta),cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi); ...
    -sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi)];
%% Given Points
h = 167;
point1_platform = [-9.92, 56.28, h];
point2_platform = [9.93, 56.28, h];
point3_platform = [53.7, -19.55, h];
point4_platform = [43.78, -36.74, h];
point5_platform = [-43.78, -36.74, h];
point6_platform = [-53.7, -19.55, h];
points_platform = [point1_platform;point2_platform;point3_platform;...
    point4_platform;point5_platform;point6_platform];
point1_base = [-53.51, 44.9, 0];
point2_base = [53.51, 44.9, 0];
point3_base = [65.64, 23.89, 0];
point4_base = [12.13, -68.79, 0];
point5_base = [-12.13, -68.79, 0];
point6_base = [-65.54, 23.89, 0];
points_base = [point1_base;point2_base;point3_base;...
    point4_base;point5_base;point6_base];

%% Result
fprintf('** Result **\n')
figure()
view(10, 10)
hold on
for i = 1 : length(points_base) - 1 
    v1 = [points_platform(i,:); points_platform(i+1,:)];
    v2 = [points_base(i,:); points_base(i+1,:)];
    v3 = [points_platform(i,:); points_base(i,:)];
    fprintf('Actuator %d initial length: %d mm\n', i, pdist(v3,'euclidean'))
    plot3(v1(:,1),v1(:,2),v1(:,3),'k')
    plot3(v2(:,1),v2(:,2),v2(:,3),'k')
    plot3(v3(:,1),v3(:,2),v3(:,3),':k')
    points_platform_h_i = points_platform(i,:)-[0,0,h];
    points_platform_h_ip1 = points_platform(i+1,:)-[0,0,h];
    vv1 = [points_platform_h_i*rotationMatrix+[0,0,h]; points_platform_h_ip1*rotationMatrix+[0,0,h]];
    plot3(vv1(:,1),vv1(:,2),vv1(:,3),'r')
    vv2 = [points_platform_h_i*rotationMatrix+[0,0,h]; points_base(i,:)];
    fprintf('Actuator %d final length  : %d mm\n', i, pdist(vv2,'euclidean'))
    plot3(vv2(:,1),vv2(:,2),vv2(:,3),'--r')
end
i = length(points_base);
v1 = [points_platform(i,:); points_platform(1,:)];
v2 = [points_base(i,:); points_base(1,:)];
plot3(v1(:,1),v1(:,2),v1(:,3),'k')
plot3(v2(:,1),v2(:,2),v2(:,3),'k')
v3 = [points_platform(i,:); points_base(i,:)];
plot3(v3(:,1),v3(:,2),v3(:,3),':k')
fprintf('Actuator %d initial length: %d mm\n', i, pdist(v3,'euclidean'))
points_platform_h_i = points_platform(i,:)-[0,0,h];
points_platform_h_ip1 = points_platform(1,:)-[0,0,h];
vv1 = [points_platform_h_i*rotationMatrix+[0,0,h]; points_platform_h_ip1*rotationMatrix+[0,0,h]];
plot3(vv1(:,1),vv1(:,2),vv1(:,3),'r')
vv2 = [points_platform_h_i*rotationMatrix+[0,0,h]; points_base(i,:)];
fprintf('Actuator %d final length  : %d mm\n', i, pdist(vv2,'euclidean'))
plot3(vv2(:,1),vv2(:,2),vv2(:,3),'--r')