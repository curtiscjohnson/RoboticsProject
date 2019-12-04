function S1Obj = shape(input)
%SHAPE Generate patched object from point cloud
%   Detailed explanation goes here
X = input(:,1);
Y = input(:,2);
Z = input(:,3);
faces = boundary(X,Y,Z);
S1Obj = patch('Faces',faces,'Vertices',input,'visible','off'); 
end

