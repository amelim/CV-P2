function g=vestaGravCart(x,y,z)
r=sqrt(x^2+y^2+z^2);
long=atan2(y,x);
long=long*180/pi;
lat=atan(z/sqrt(x^2+y^2))*180/pi;
g=vestaGrav(r,lat,long);
end