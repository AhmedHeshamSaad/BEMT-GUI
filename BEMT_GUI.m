function varargout = BEMT_GUI(varargin)
% BEMT_GUI MATLAB code for BEMT_GUI.fig
%      BEMT_GUI, by itself, creates a new BEMT_GUI or raises the existing
%      singleton*.
%
%      H = BEMT_GUI returns the handle to a new BEMT_GUI or the handle to
%      the existing singleton*.
%
%      BEMT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BEMT_GUI.M with the given input arguments.
%
%      BEMT_GUI('Property','Value',...) creates a new BEMT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BEMT_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BEMT_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BEMT_GUI

% Last Modified by GUIDE v2.5 23-Jul-2018 12:34:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BEMT_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @BEMT_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before BEMT_GUI is made visible.
function BEMT_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BEMT_GUI (see VARARGIN)

% Choose default command line output for BEMT_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes BEMT_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BEMT_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ImportButton.
function ImportButton_Callback(hObject, eventdata, handles)
% hObject    handle to ImportButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Airfoil

% Laod Airfoil Data .txt
[name,path] = uigetfile('*','Select Aitfoil txt file:');
data = load(strcat(path,name));
Airfoil.x = data(:,1);   % x coordinates
Airfoil.y = data(:,2);   % y coordinates
Airfoil.z = zeros(length(Airfoil.x),1);   % z coordinates of zeroes - 2D 

axes(handles.F1)
if get(handles.HoldCB,'Value')
    hold on
end

plot(Airfoil.x,Airfoil.y)

P1 = get(handles.F1,'Position');
P2 = get(handles.figure1,'Position');
P3 = get( groot, 'Screensize');
x = P1(3)*P2(3)*P3(3);
y = P1(4)*P2(4)*P3(4);
xlim([0 1*(1-(y-x)/x)]), ylim([-0.5 0.5])

set(handles.F1,'YMinorGrid','on','XMinorGrid','on')
set(handles.F1,'YMinorTick','on','XMinorTick','on')
hold off


% --- Executes on button press in ExportButton.
function ExportButton_Callback(hObject, eventdata, handles)
% hObject    handle to ExportButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Sec r c theta

% Export Desgin points into excel file 
Header = {'r','Pitch angle','chord length'};
Table = [r,theta,c];
[name,path] = uiputfile('DesginPoints.xlsx','Save');
xlswrite(strcat(path,name),Header,1,'A1')
xlswrite(strcat(path,name),Table,1,'A2')

% Export Flat Airfoils sections
for j = 1:length(Sec.x)
    if get(handles.FlatRB,'Value')     % Flat
        file = fopen(strcat(path,sprintf('Flatsec%.0f.txt',j)),'w');
    else                               % Curved
        file = fopen(strcat(path,sprintf('Curvedsec%.0f.txt',j)),'w');
    end
    
    % Write airfoil coordinates
    if get(handles.CADmenu,'Value') == 1        % SoildWorks
        for i = 1:length(Sec.x{j})  % For each Section
            fprintf(file,'%f\t%f\t%f\r\n',Sec.x{j}(i),Sec.y{j}(i),Sec.z{j}(i));
        end
    elseif get(handles.CADmenu,'Value') == 2    % DesignModeler 
        for i = 1:length(Sec.x{j})+1  % For each Section
            if i == length(Sec.x{j})+1     % Last line
                fprintf(file,'1\t0');
            else
                fprintf(file,'%g\t%g\t%f\t%f\t%f\r\n', 1, ...
                    i,Sec.x{j}(i),Sec.y{j}(i),Sec.z{j}(i));
            end
        end
    end
    
    fclose(file);
end
msgbox('Done!')


function TSR_Callback(hObject, eventdata, handles)
% hObject    handle to TSR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TSR as text
%        str2double(get(hObject,'String')) returns contents of TSR as a double


% --- Executes during object creation, after setting all properties.
function TSR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TSR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function CL_Callback(hObject, eventdata, handles)
% hObject    handle to CL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CL as text
%        str2double(get(hObject,'String')) returns contents of CL as a double


% --- Executes during object creation, after setting all properties.
function CL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function B_Callback(hObject, eventdata, handles)
% hObject    handle to B (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of B as text
%        str2double(get(hObject,'String')) returns contents of B as a double


% --- Executes during object creation, after setting all properties.
function B_CreateFcn(hObject, eventdata, handles)
% hObject    handle to B (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AOA_Callback(hObject, eventdata, handles)
% hObject    handle to AOA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AOA as text
%        str2double(get(hObject,'String')) returns contents of AOA as a double


% --- Executes during object creation, after setting all properties.
function AOA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AOA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Callback(hObject, eventdata, handles)
% hObject    handle to R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R as text
%        str2double(get(hObject,'String')) returns contents of R as a double


% --- Executes during object creation, after setting all properties.
function R_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function N_Callback(hObject, eventdata, handles)
% hObject    handle to N (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of N as text
%        str2double(get(hObject,'String')) returns contents of N as a double


% --- Executes during object creation, after setting all properties.
function N_CreateFcn(hObject, eventdata, handles)
% hObject    handle to N (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in CADmenu.
function CADmenu_Callback(hObject, eventdata, handles)
% hObject    handle to CADmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns CADmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from CADmenu


% --- Executes during object creation, after setting all properties.
function CADmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CADmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IR_Callback(hObject, eventdata, handles)
% hObject    handle to IR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IR as text
%        str2double(get(hObject,'String')) returns contents of IR as a double


% --- Executes during object creation, after setting all properties.
function IR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in SolveButton.
function SolveButton_Callback(hObject, eventdata, handles)
% hObject    handle to SolveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Airfoil Sec r c theta

TSR = str2double(get(handles.TSR,'String'));    % Tip Speed Ratio
CL = str2double(get(handles.CL,'String'));      % Lift Coefficient
B = str2double(get(handles.B,'String'));        % Number of Blades
AOA = str2double(get(handles.AOA,'String'));    % Angle of Attack
R = str2double(get(handles.R,'String'));        % Rotor Radius
IR = str2double(get(handles.IR,'String'));      % Radius where blade start
N = str2double(get(handles.N,'String'));        % number of requested sections

dr = (R - IR) / (N-1);  % distance between sections

% create working parameters
r = IR:dr:R;            % currunt section radius
Lr = TSR*r/R;        % speed radio

% Calculate pitch angle and cord length
% next equations are from Wind Energy Explained, Manwell 2nd Ed
% equations 3.105 and 3.106 to calculate relative angle and chord 
% length, respectively.
if get(handles.wWakeRB,'Value')                 % w/ wake
    phi = 2/3 * atan(1./Lr) * 180/pi; 
    c = 8*pi*r.*(1-cos(phi*pi/180))/(B*CL); 
else                                            % w/out wake
    phi = atan(2./(3*Lr)) * 180/pi; 
    c = 8*pi*r.*sin(phi*pi/180)./(3*B*CL*Lr); 
end
theta = phi - AOA;  % Pitch Angle

% Scale and rotate airfoil at every section
Sec.x = []; 
Sec.y = [];
Sec.z = [];
axes(handles.F2)
if get(handles.HoldCB,'Value')
    hold on
end

for j = 1:N
    % scale arifoil and move it to moment center (quarter chord)
    Sec.x{j} = Airfoil.x * c(j) - 0.25*c(j); 
    Sec.y{j} = Airfoil.y * c(j);
    Sec.z{j} = Airfoil.z + r(j);

    % Rotate airfoil with given pitch angle
    v = [Sec.x{j}';Sec.y{j}'];
    th = theta(j)*pi/180;
    Rv = [cos(th) -sin(th); sin(th) cos(th)];
    vR = Rv*v;
    Sec.x{j} = vR(1,:)';
    Sec.y{j} = vR(2,:)';

    if ~get(handles.FlatRB,'Value')   % if Curved 
        % curve airfoil
        for i = 1:length(Sec.x{j})
            angle = Sec.x{j}(i)/r(j);            % determine point angle
            Sec.x{j}(i) = r(j) * sin(angle);     % curved x point
            Sec.z{j}(i) = r(j) * cos(angle);     % curved z point
        end
    end

    % change axis of rotation from y to z
    a = Sec.y{j};
    Sec.y{j} = -Sec.z{j};
    Sec.z{j} = a;

    % plot sections
    plot3(Sec.x{j},Sec.y{j},Sec.z{j})
    xlim([-R/2 R/2]), ylim([-R 0]), zlim([-R/2 R/2])
    hold on
end

hold off



% --- Executes on button press in EXCondButton.
function EXCondButton_Callback(hObject, eventdata, handles)
% hObject    handle to EXCondButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Export Desgin points into excel file 
TSR = str2double(get(handles.TSR,'String'));    % Tip Speed Ratio
CL = str2double(get(handles.CL,'String'));      % Lift Coefficient
B = str2double(get(handles.B,'String'));        % Number of Blades
AOA = str2double(get(handles.AOA,'String'));    % Angle of Attack
R = str2double(get(handles.R,'String'));        % Rotor Radius
IR = str2double(get(handles.IR,'String'));      % Radius where blade start
N = str2double(get(handles.N,'String'));        % number of requested sections

[name,path] = uiputfile('DesignConditions.xlsx','Save');
par = {'TSR','Cl','B','AOA','R','IR','N'};
values = [TSR;CL;B;AOA;R;IR;N];
xlswrite(strcat(path,name),par',1,'A1')
xlswrite(strcat(path,name),values,1,'B1')
msgbox('Done!')


% --- Executes on button press in HoldCB.
function HoldCB_Callback(hObject, eventdata, handles)
% hObject    handle to HoldCB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of HoldCB


% --- Executes on button press in HoldCB.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to HoldCB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of HoldCB
