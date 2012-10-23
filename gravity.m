function u = gravity(r,lat,long,M,R,Cfts)
%r - satellite orbit radius (meters)
%lat - satellite latitude (deg)
%long - satellite longitude (deg)
%M - mass of planetoid (kg)
%R - reference radius of planetoid (meters)
%Cfts - 2x((n^2+3n)/2) matrix of normalized coefficients

G=6.67384*10^(-11);
n=(-3+sqrt(9+8*length(Cfts)))/2;
C=Cfts(1,:);
S=Cfts(2,:);

u=1;
for i=1:n
    P=legendre(i,sind(lat),'norm');
    for j=0:i
        k=(i-1)*(2+i)/2+j+1;
        u=u+(R/r)^i*P(j+1)*(C(k)*cosd(j*long)+S(k)*sind(j*long));
    end
end
u=u*G*M/r;
end