T0 = [[1,0,0,3];[0,1,0,3];[0,0,1,3];[0,0,0,1]];
T1 = angvec2tr(pi/2,[1,0,0]);
T1=T0*T1;
T1(1,4)=1;
Trr=ctraj(T0, T1, 100);
a=zeros(100,1);
for i = 1:1:100
    a(i)=Trr(1,4,i);
end
t=[1:1:100]
