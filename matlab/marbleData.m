% data=csvread('/home/jens/UNI/project/rb-rca5/build-robot_control-Desktop-Debug/marbleData.txt');
data=csvread('marbleData320x240.txt');
mx=data(:,1);
my=data(:,2);
md=data(:,3);
ma=data(:,4);
mdx=data(:,5);
mdy=data(:,6);
mdd=data(:,7);
mda=data(:,8);

diff=sqrt((mx-mdx).^2+(my-mdy).^2);
under05=sum(diff<=0.5)/length(diff)

[ii,i2,i2] = unique(md);
[jj,j2,j2] = unique(ma);
Z = accumarray([j2,i2],diff,[],[],0);

figure(1)
surf(ii,jj,Z)

angDiff=ma-mda;

meana=mean(angDiff)*(pi/180.0);

A = accumarray(j2,angDiff);

distDiff=md-mdd;
meand=mean(distDiff);

figure(2)
histogram (diff,50) 
figure(3)
histogram (distDiff,50)  
figure(4)
histogram (angDiff*(pi/180.0),50)  