%Name: collisiondetect.m
%Author: Arnold Wright
%Date: 2019-11-19
% Dependencies: MATLAB-GJK-Collision-Detection by Matthew Sheen (github:mws262)
%               Baxter model (BYU: ME537)
function [collide] = collisiondetect(q, sphere)
% Output: Bool: 0 = no collision, 1 = collision
% Input: q as vector in radians, sphere as [x y z r] in mm

%Check Variables:
if size(q,2) ~= 1
    fprintf("Correct format for q:[q1 q2 q3 q4 q5 q6 q7]");
end
if size(sphere,1) ~= 4 || size(sphere,2) ~= 1
    fprintf("Correct format for sphere:[x y z r]");
end

collide = 0;

%%% BAXTER DATA %%%
% Arm Segment Collision Cylinders
%        zlength radius (both mm)
Arms(1,:) = [100 100];
Arms(2,:) = [100 100];
Arms(3,:) = [100 100];
Arms(4,:) = [100 100];
Arms(5,:) = [100 100];
Arms(6,:) = [100 100];
Arms(7,:) = [100 100];
% Chest Collision Box
%        x   y   z
Body = [100 100 100;...
        100 100 100;...
        100 100 100;...
        100 100 100;...
        100 100 100;...
        100 100 100];
% Floor Collision Plane
%         x   y   z
Floor = [100 100 100;...
         100 100 100;...
         100 100 100;...
         100 100 100];
numiterations = 6;

%%% CONVERT TO POINT CLOUDS %%%
if ~exist('armcloud',1) %Be careful with this, but might speed it up
    for i = 1:7
        % circle of desired radius in base frame
        
        % copy it by cylinder length
        
        % rotate it into the arm frame
        
        % end format: [x1 y1 z1; x2 y2 z2; ....]
        armcloud(i,:) = 0;
    end
end
if ~exist('spherecloud',1)
    
end

%%% COLLISIONS %%%
% Arms 2-7 with body:
for i = 2:7
    temp1 = GJK(armcloud(i,:),body,numiterations);
end

% Arms 2-7 with sphere:
for i = 2:7
    temp1 = GJK(armcloud(i,:),sphere,numiterations);
end

% Arms 5-7 with floor:
for i = 5:7
    temp3 = GJK(armcloud(i,:),floor,numiterations);
end

% Arms 4-7 with arms 3+ less than them
for i = 4:7
    for j = (i-3):-1:1
        temp4 = GJK(armcloud(i,:),armcloud(j,:),numiterations);
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

