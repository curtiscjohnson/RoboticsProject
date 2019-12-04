clear all;
close all;
clc;

% q = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
sphere = [1 1 1 0.5];
[left,right] = mdl_baxter('');
num = 100;
p = 0;
tic
for i = 1:2
    for j = 1:2
        for k = 1:2
            for l = 1:2
                for m = 1:2
                    for n = 1:2
                        for o = 1:2
                            c = collisiondetect([i,j,k,l,m,n,o],sphere,right);
                            fprintf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f :%.f\n",i,j,k,l,m,n,o,c);
                            p = p+1;
                        end
                    end
                end
            end
        end
    end
end
toc
