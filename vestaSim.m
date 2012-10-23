clc;
clear all;

v=[50000 0 120000]; %m/s
p=[0 500000 0]; %meters
w=0.01871958; %deg/s


dt=.1;

for i=1:1000
    g=vestaGravCart(p(i,1),p(i,2),p(i,3));
    v(i+1,:)=v(i,:)-g*dt*p(i,:)/norm(p(i,:));
    p(i+1,:)=p(i,:)+v(i,:)*dt;
    p(i+1,:)=[cosd(w) -sind(w) 0; sind(w) cosd(w) 0; 0 0 1]*p(i+1,:)';
end

plot3(p(:,1),p(:,2),p(:,3));
hold on;
[X Y Z]=sphere(50);
for i=1:length(X(:))
    
X=265000*X;
Y=265000*Y;
Z=265000*Z;
surf(X,Y,Z);
axis equal;
