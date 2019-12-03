right.plot(q); 
[T,A] = right.fkine(q);
hold on; 
for i=1:7
    trplot(A(i),'frame',num2str(i))
end