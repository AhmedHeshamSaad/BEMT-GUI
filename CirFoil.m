% Clear memory
clear; clc; close all;

% Laod Airfoil Data .txt
[name,path] = uigetfile('*','Select Aitfoil txt file:');
data = load(strcat(path,name));
xdat = data(:,1);   % x coordinates
ydat = data(:,2);   % y coordinates
zdat = ones(length(xdat),1);   % z coordinates of zeroes - 2D 

x = xdat - 0.25; 
y = ydat;
z = zdat;

c = 1;  % chord length
s = 1;  % local span
theta = zeros(length(x),1);
xc = zeros(length(x),1);
zc = zeros(length(x),1);
r = s;
for i = 1:length(x)
    theta(i) = x(i)/r;
    xc(i) = r * sin(theta(i));
    zc(i) = r * cos(theta(i));
end

plot3(x,y,z), hold on
plot3(xc, ydat, zc)
zlim([0 1]), xlim([-0.25 0.75]), ylim([-0.5 0.5])
xlabel('X'), ylabel('Y'),zlabel('Z')