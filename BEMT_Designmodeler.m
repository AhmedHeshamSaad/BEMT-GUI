% Using BEM Theory to obtaion various sections of HAWT balde(flated and curved)
%----------------------------------------------------------------------
% First, Make sure to select the airfoil data file with the first row
% is the first airfoil coordinats (delete any discription lines above it).
% Otherwise you will recieve an error message.
% ---------------------------------------------------------------------
% Design Parameters:
%     Tip Speed ratio     >> usually 6 
%     Lift coefficient    >> corresponding to maximum Cl/Cd
%     No of blades        >> numbers of turbine blades 
%     Angle of Attack     >> corresponding to maximum Cl/Cd
%     Radius              >> radius of turbine
%     intial radius       >> radius where the strat of the blade
% ----------------------------------------------------------------------   
% Warning : On running this the workspace memory will be deleted. Save if
% any data present before running the code !!
% ----------------------------------------------------------------------
% Code Witten By 
%                     Ahmed Hesham Saad
%         email:      ahmed.h.saad.y@gmail.com
% -----------------------------------------------------------------------

% Clear memory
clear; clc; close all;

% Laod Airfoil Data .txt
[name,path] = uigetfile('*','Select Aitfoil txt file:');
data = load(strcat(path,name));
xdat = data(:,1);   % x coordinates
ydat = data(:,2);   % y coordinates
zdat = zeros(length(xdat),1);   % z coordinates of zeroes - 2D 

% Get Design Parameters
design_parameters = inputdlg({'Tip Speed Ratio (TSR)=',...
    'Lift Coffecient (CL)=','No of blades','Angle of attack(deg) =',...
    'Radius(m)=','Initial radius(m)=','No of sections ='},...
    'Design Parameters');
TSR = str2double(design_parameters{1});     % Tip Speed Ratio
cl = str2double(design_parameters{2});      % Lift Coefficient
B = str2double(design_parameters{3});       % Number of Blades
AOA = str2double(design_parameters{4});     % Angle of Attack
R = str2double(design_parameters{5});       % Rotor Radius
IR = str2double(design_parameters{6});      % Radius where blade start
N = str2double(design_parameters{7});       % number of requested sections

dr = (R - IR) / (N-1);  % distance between sections

% create working parameters
r = zeros(N,1);         % currunt section radius
Lr = zeros(N,1);        % speed radio
phi = zeros(N,1);       % Relative angle
theta = zeros(N,1);     % Pitch angle
c = zeros(N,1);         % Chord Length

% Creat loop for every section
for i = 1:N
    if (i == 1)
        r(i) = IR;  % intial radius
    else
        r(i) = r(i-1) + dr; % next raduis
    end
    % next equation are from Wind Energy Explained, Manwell 2nd Ed
    % equations 3.105 and 3.106 to calculate relative angle and chord 
    % length, respectively.
    Lr(i) = TSR*r(i)/R; 
    phi(i) = 2/3 * atan(1/Lr(i)) * 180/pi; % w/ wake
%     phi(i) = atan(2/(3*Lr(i))) * 180/pi; % w/out wake
    theta(i) = phi(i) - AOA;
    c(i) = 8*pi*r(i)*(1-cos(phi(i)*pi/180))/(B*cl); % w/ wake
%     c(i) = 8*pi*r(i)*sin(phi(i)*pi/180)/(3*B*cl*Lr(i)); % w/out wake
end

% Export Desgin points into excel file 
Header = {'r','Pitch angle','chord length'};
Table = [r,theta,c];
[name,path] = uiputfile('DesginPoints.xlsx','Save');
xlswrite(strcat(path,name),Header,1,'A1')
xlswrite(strcat(path,name),Table,1,'A2')

par = {'TSR','Cl','B','AOA','R','IR','N'};
values = [TSR;cl;B;AOA;R;IR;N];
xlswrite(strcat(path,name),par',1,'E1')
xlswrite(strcat(path,name),values,1,'F1')

% Export Flat Airfoils sections
group = ones(length(xdat),1);
point = 1:length(xdat);
for j = 1:N
    % scale arifoil and move it to moment center
    x = xdat * c(j) - 0.25*c(j); 
    y = ydat * c(j);
    z = zdat + r(j);
    
    % Rotate airfoil with given pitch angle
    v = [x';y'];
    th = theta(j)*pi/180;
    R = [cos(th) -sin(th); sin(th) cos(th)];
    vR = R*v;
    x = vR(1,:)';
    y = vR(2,:)';
    
    % change axis of rotation from y to z
    a = y;
    y = -z;
    z = a;

    file = fopen(sprintf('sec%.0f-flat-DM.txt',j),'w');
    for i = 1:length(x)+1
        if i < length(x)+1
            fprintf(file,'%g\t%g\t%f\t%f\t%f\r\n',group(i), ...
                point(i),x(i),y(i),z(i));
        elseif i == length(x)+1
            fprintf(file,'1\t0');
        end
    end
    fclose(file);
    
    % plot sections
    plot3(x,y,z), hold on
end

% Export Curved Airfoils sections
angle = zeros(length(x),1);
xc = zeros(length(x),1);
zc = zeros(length(x),1);
for w = 1:N
    file = fopen(sprintf('sec%.0f-curved-DM.txt',w),'w');
    % scale arifoil and move it to moment center
    x = xdat * c(w) - 0.25*c(w); 
    y = ydat * c(w);
    z = zdat + r(w);
    
    % Rotate airfoil with given pitch angle
    v = [x';y'];
    th = theta(w)*pi/180;
    R = [cos(th) -sin(th); sin(th) cos(th)];
    vR = R*v;
    x = vR(1,:)';
    y = vR(2,:)';
    
    % curve airfoil
    for q = 1:length(x)
        angle(q) = x(q)/r(w);          % determine point angle
        xc(q) = r(w) * sin(angle(q));  % curved x point
        zc(q) = r(w) * cos(angle(q));  % curved z point
    end
    yc = y;
    
    % change axis of rotation from y to z
    a = yc;
    yc = -zc;
    zc = a;
    
    %plot sections
    for i = 1:length(x)
        fprintf(file,'%f\t%f\t%f\r\n',xc(i),yc(i),zc(i));
    end
    fclose(file);
    plot3(xc,yc,zc), hold on
end



