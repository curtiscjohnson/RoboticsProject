function S1Obj = shape(input)
%SHAPE Generate patched object from point cloud
%   Detailed explanation goes here
X = input(:,1);
Y = input(:,2);
Z = input(:,3);
faces = boundary(X,Y,Z,0);
% Enable line below for visualization, disable for faster calculation
%S1Obj = patch('Faces',faces,'Vertices',input,'FaceVertexCData',jet(size(input,1)),'FaceColor','interp'); 
% Line below is fast calculation line.  Disable to visualize.
S1Obj = patch('Faces',faces,'Vertices',input,'visible','off');
end

