clc
clear
%data=csvread('/home/jens/UNI/project/rb-rca5/build-robot_control-Desktop-Debug/marbleData.txt');
data=csvread('marbleData2.txt');
mx=data(:,1);
my=data(:,2);
md=data(:,3);
ma=data(:,4);
mdx=data(:,5);
mdy=data(:,6);
mdd=data(:,7);
mda=data(:,8);


diff=sqrt((mx-mdx).^2+(my-mdy).^2);
distDiff=md-mdd;
angDiff=ma-mda;
%

percentagefound=length(data(:,1))/3000
under05=sum(diff<=0.5)/length(diff)
% 

figure(1)
histogram (diff,50,'BinLimitsMode','auto') 
xlabel('Position difference [m]');
xlim([0 2])
posmean=mean(diff)
posvariance=var(diff)

figure(2)
histogram (distDiff,50)  
xlabel('Distance diffrence[m]');
figure(3)
 histogram (angDiff*(pi/180.0),50);
 xlim([-0.003 0.003])
xlabel('Angle difference [degree]');
angmean=mean(diff)
angvariance=var(diff)






