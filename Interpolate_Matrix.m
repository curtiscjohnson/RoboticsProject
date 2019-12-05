%A is the input matrix. It should be able to be any size
%N is the number of points you want between each value

function [Ipoints]=Interpolate_Matrix(A,N)

%sizing for the matlab function
sz=size(A);
row=sz(1);
clmn=sz(2);

step=1/N;
p=(1:step:row);

%interpolation function takes in a matrix of existing points (A) and a
%vector of desired points (p). We can change spline to pretty much anything
%we want. linear, cubic, etc.
Ipoints=interp1(A,p,'spline');

%plot if you want. Not very refined though
% plot(Interpoints,'o')

end



