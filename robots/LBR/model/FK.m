% robotROSMessageUsageExample - ROS examples
% robotROSFeedbackControlExample
% Denavit-Hartenberg link parameters for the 7-link kuka iiwa 
n = 7;               %number of links
a = sym([0; 0; 0; 0; 0; 0; 0]);
alp = sym([pi/2; -pi/2; pi/2; -pi/2; pi/2; -pi/2; 0;]);

% d01 = 0.36 | d23 = 0.42 | d45 = 0.4 | d67 = 0.126
syms d01 d23 d45 d67;
d = [d01; 0; d23; 0; d45; 0; d67];

syms th1 th2 th3 th4 th5 th6 th7;
th = [th1; th2; th3; th4; th5; th6; th7];

H = sym('H',[4 4 n]);
for i = 1:n
H(:,:,i) = [cos(th(i))  -sin(th(i))*cos(alp(i))  sin(th(i))*sin(alp(i))  a(i)*cos(th(i)); ... 
            sin(th(i))   cos(th(i))*cos(alp(i)) -cos(th(i))*sin(alp(i))  a(i)*sin(th(i)); ...
               0              sin(alp(i))              cos(alp(i))             d(i);      ...
               0                  0                        0                     1        ];
end;

H0i = sym('H0i',[4 4 n]);
for i = 1:n
    H0i(:,:,i) = sym(eye(4));
    for j = 1:i
        %HJ = double(subs((H(:,:,j)),{th(j),d01,d23,d45,d67},{0,0.36,0.42,0.4,0.126}));
        H0i(:,:,i) = H0i(:,:,i)*H(:,:,j);
    end;
end;

% Расчет якобианов для всех систем координат системы
J = sym(zeros(6,n,n));
zjj = sym([0;0;1]);
pjj = sym([0;0;0]);
for i = 1:n
    for j = 1:i
        if j==1
            z0im1 = zjj;
            p0im1 = pjj;
        else
            z0im1 = H0i(1:3,1:3,j-1)*zjj;
            p0im1 = H0i(1:3,4,j-1);
        end
        J(:,j,i) = [cross(z0im1,H0i(1:3,4,i)-p0im1);z0im1];
    end
end;

J7t = transpose(J(:,:,7));

H0N = H(:,:,1)*H(:,:,2)*H(:,:,3)*H(:,:,4)*H(:,:,5)*H(:,:,6)*H(:,:,7);
HC = double(subs(H0N,{th1,th2,th3,th4,th5,th6,th7,d01,d23,d45,d67},{0,0,0,0,0,0,0,0.36,0.42,0.4,0.126}));
%tform2eul(HC);

%rosinit
%tftree = rostf

%timepub = rospublisher('/timer', 'std_msgs/Time')
%pause(2)
%timemsg = rosmessage(timepub);
%timemsg.Data = rostime('now')
