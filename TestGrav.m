clc;
clear all;

r=500000;
lat=-90:5:90;
long=-180:5:180;
for i=1:length(long)
    for j=1:length(lat)
        g(j,i)=vestaGrav(r,lat(j),long(i));
    end
end
[LONG LAT]=meshgrid(long,lat);
pcolor(LONG,LAT,g);