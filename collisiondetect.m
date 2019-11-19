%Name: collisiondetect.m
%Author: Arnold Wright
%Date: 2019-11-19

%Variables:


%Code:

% Planned Pseudocode:
%{
[boolean:collide] = (1xn:q, 1x4:obstacle_position,radius[x y z r])
Take in a vector of q's and the 3D position and radius of the object

Have some configuration as to the dimensions and sizes of the links

Look for an intersection of the obstacle with each link in turn

return a binary: 0 = false = no collision
                 1 = true = collision.
%}


% Notes:
%{
Option 1:
MATLAB's robotics toolbox (not Peter's) with checkCollision,
collisionCylinder, collisionSphere.

Option 2: (simpler & faster (used for real-time situations))
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

