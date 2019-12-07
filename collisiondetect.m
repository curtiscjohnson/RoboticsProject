%Name: collisiondetect.m
%Author: Arnold Wright
%Date: 2019-11-19
% Dependencies: MATLAB-GJK-Collision-Detection by Matthew Sheen (github:mws262)
%               Baxter model (BYU: ME537)
function [collide] = collisiondetect(q, spherecoords)
% Output: Bool: 0 = no collision, 1 = collision
% Input: q as vector in radians, sphere as [x y z r] in m

dbstop if error

clear armcloud  %%% DEBUG

% Convert python inputs to correct MATLAB data types
% See https://www.mathworks.com/help/matlab/matlab_external/pass-data-to-matlab-from-python.html
% list -> cell array
q = cell2mat(q);
spherecoords = cell2mat(spherecoords);


%Check Variables:
if size(q,1) ~= 1
    fprintf("Correct format for q:[q1 q2 q3 q4 q5 q6 q7]\n");
end
if size(spherecoords,1) ~= 1 || size(spherecoords,2) ~= 4
    fprintf("Correct format for sphere:[x y z r]\n");
end

collide = 0;

[left,right] = mdl_baxter('');
[T,A] = right.fkine(q);

%%% BAXTER DATA %%%
% Arm Segment Collision Cylinders
%        zlength radius (both m)
Arms(1,:) = [0.27035 0.1];
Arms(2,:) = [0.069 0.1];
Arms(3,:) = [0.36435 0.1];
Arms(4,:) = [0.069 0.1];
Arms(5,:) = [0.37429 0.1];
Arms(6,:) = [0.01 0.1];
Arms(7,:) = [0.229525 0.1];
% Chest Collision Box
%        x   y   z
body = [-0.2  0 1;...
        0.3 0 1;...
         0.3 1 1;...
         -0.2  1 1;...
        -0.2  0 -1;...
        0.3 0 -1;...
         0.3 1 -1;...
         -0.2  1 -1];

% Floor collision Plane
%         x   y   z
floor = [-2  2 -1;...
         -2 -2 -1;...
          2 -2 -1;...
          2  2 -1;...
         -2  2 -1.5;...
         -2 -2 -1.5;...
          2 -2 -1.5;...
          2  2 -1.5];
numiterations = 6;
numpoints = 20;

%%% CONVERT TO POINT CLOUDS %%%

for i = 1:7
    % circle of desired radius in base frame
    r = Arms(i,2);
    angles = linspace(0, 2*pi, numpoints+1);
    X = r * cos(angles(1:numpoints));
    Y = r * sin(angles(1:numpoints));
    Z = zeros(1,numpoints);
    % copy it by cylinder length
    X = cat(2,X,X);
    Y = cat(2,Y,Y);
    Z = cat(2,Z,Z + Arms(i,1));
    cloud = cat(1,X,Y,Z,ones(1,numpoints*2));
    % transform it into the arm frame
    if i >= 2
        for j = 1:(2 * numpoints)
            cloud(:,j) = A(i-1).T*cloud(:,j);
        end
    end
    % end format: [x1 y1 z1; x2 y2 z2; ....]
    armcloud(i,:,:) = cloud(1:3,:);
end
    x2 = spherecoords(1);
    y2 = spherecoords(2);
    z2 = spherecoords(3);
    r = spherecoords(4);
    [X,Y,Z] = sphere;
    [f,spherecloud,c] = surf2patch(X*r+x2,Y*r+y2,Z*r+z2,'triangles');
%     angles = linspace(0, 2*pi, numpoints+1);
%     X = r * cos(angles(1:numpoints));
%     Y = r * sin(angles(1:numpoints));
%     Z = z2 * ones(1,numpoints);
%     scale = sqrt(3)/2;
%     X1 = cat(2,X+x2,scale*X+x2);
%     Y1 = cat(2,Y+y2,scale*Y+y2);
%     Z1 = cat(2,Z,Z + 0.5*r);
%     X1 = cat(2,X1,scale*X+x2);
%     Y1 = cat(2,Y1,scale*Y+y2);
%     Z1 = cat(2,Z1,Z - 0.5*r);
%     X1 = cat(2,X1,x2,x2);
%     Y1 = cat(2,Y1,y2,y2);
%     Z1 = cat(2,Z1,z2+r,z2-r);
%     spherecloud = [X1;Y1;Z1].';
%%%DEBUG
% X = 0; Y = 0; Z = 0;
% for i = 1:7
%     X1 = squeeze(armcloud(i,1,:)).';
%     Y1 = squeeze(armcloud(i,2,:)).';
%     Z1 = squeeze(armcloud(i,3,:)).';
%     X = [X X1];
%     Y = [Y Y1];
%     Z = [Z Z1];
% end
% plot3(X,Y,Z)
%%% END DEBUG


%%% COLLISIONS %%%
% Arms 4-7 with body:
for i = 4:7
    temp1 = GJK(shape(squeeze(armcloud(i,:,:)).'),shape(body),numiterations);
end

% Arms 3-7 with sphere:
%for i = 3:7
%    temp2 = GJK(shape(squeeze(armcloud(i,:,:)).'),shape(spherecloud),numiterations);
%end

% Arms 5-7 with floor:
for i = 5:7
    temp3 = GJK(shape(squeeze(armcloud(i,:,:)).'),shape(floor),numiterations);
end

% Arms 5-7 with arms 3+ less than them
for i = 4:7
    for j = (i-3):-1:1
        temp4 = GJK(shape(squeeze(armcloud(i,:,:)).'),shape(squeeze(armcloud(j,:,:)).'),numiterations);
    end
end

% return collide as bool 0/1
collide = temp1 + temp2 + temp3 + temp4;
if collide > 1
    collide = 1;
end
end

% Notes:
%{
File Exchange GJK algorithm (minimum distance between two convex objects)
Psuedocode:
 function GJK_intersection(shape p, shape q, vector initial_axis):
       vector  A = Support(p, initial_axis) - Support(q, -initial_axis)
       simplex s = {A}
       vector  D = -A
       loop:
           A = Support(p, D) - Support(q, -D)
           if dot(A, D) < 0:
              reject
           s = s ∪ A
           s, D, contains_origin = NearestSimplex(s)
           if contains_origin:
              accept
Support(shape,d) returns the point on shape which has the highest dot product with d
NearestSimplex(s) takes a simplex and returns the simplex on s closests to the
origin, and a direction toward the origin normal to the new simplex.  If s
contains the origin, NS accepts s and the two shapes are determined to
intersect.

Implemented in File Exchange: "Fast 3D Collision Detection -- GJK
algorithm" by Matthew Sheen. 
%}

