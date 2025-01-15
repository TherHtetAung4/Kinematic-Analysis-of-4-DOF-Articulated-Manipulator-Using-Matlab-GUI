%GUI for 4-DOF Articulated Manipulator

function varargout = GUI_Test1(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_Test1_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_Test1_OutputFcn, ...
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

function GUI_Test1_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;

axes(handles.axes3)
imshow('D:\Thesis excavator\mtu_logo.png')

guidata(hObject, handles);

function varargout = GUI_Test1_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

function Theta_1_Callback(hObject, eventdata, handles)

function Theta_1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_2_Callback(hObject, eventdata, handles)

function Theta_2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_3_Callback(hObject, eventdata, handles)

function Theta_3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Theta_4_Callback(hObject, eventdata, handles)

function Theta_4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Pos_X_Callback(hObject, eventdata, handles)

function Pos_X_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Pos_Y_Callback(hObject, eventdata, handles)

function Pos_Y_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Pos_Z_Callback(hObject, eventdata, handles)

function Pos_Z_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Phi_Callback(hObject, eventdata, handles)

function Phi_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in btn_Forward.
function btn_Forward_Callback(hObject, eventdata, handles)

%Forward Kinematic
Th_1 = str2double(handles.Theta_1.String)*pi/180;
Th_2 = str2double(handles.Theta_2.String)*pi/180;
Th_3 = str2double(handles.Theta_3.String)*pi/180;
Th_4 = str2double(handles.Theta_4.String)*pi/180;

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

% Define joint angle limits (in radians)
Th1_min = deg2rad(-90); Th1_max = deg2rad(90);
Th2_min = deg2rad(15); Th2_max = deg2rad(45);
Th3_min = deg2rad(-130); Th3_max = deg2rad(-60);
Th4_min = deg2rad(-90); Th4_max = deg2rad(-25);

% Check if the calculated joint angles are within limits
if (Th_1 < Th1_min || Th_1 > Th1_max) || ...
   (Th_2 < Th2_min || Th_2 > Th2_max) || ...
   (Th_3 < Th3_min || Th_3 > Th3_max) || ...
   (Th_4 < Th4_min || Th_4 > Th4_max)
   
    % Display error message in the GUI's error text box
    set(handles.error_msg, 'String', ['Error: Joint angles ' ...
        'are out of range!'],'ForegroundColor',[1 0 0]);
    
    % Exit the function if there is an error
    return;
else
    % Clear the error message if angles are within range
    set(handles.error_msg, 'String', '');
end

%Simulation Plot of 4DOF Manipulator
Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';
axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
M = T.T();
handles.Pos_X.String = num2str(floor(M(1,4)));
handles.Pos_Y.String = num2str(floor(M(2,4)));
handles.Pos_Z.String = num2str(floor(M(3,4)));
handles.Phi.String = num2str(floor((Th_2+Th_3+Th_4)*180/pi));

%Servo Write Program for Forward Push Button
clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

function btn_Inverse_Callback(hObject, eventdata, handles)

%Inverse Kinematics
dx = str2double(handles.Pos_X.String);
dy = str2double(handles.Pos_Y.String);
dz = str2double(handles.Pos_Z.String);
phi = str2double(handles.Phi.String)*pi/180;


L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';

% Define joint angle limits (in radians)
Th1_min = deg2rad(-90); Th1_max = deg2rad(90);
Th2_min = deg2rad(15); Th2_max = deg2rad(45);
Th3_min = deg2rad(-130); Th3_max = deg2rad(-60);
Th4_min = deg2rad(-90); Th4_max = deg2rad(-25);
% Calculate theta1
Th_1 = atan2(dy, dx);

% Calculate a, b, c, and r
A = (dx - L_4 * cos(phi));
B = (dy - L_4 * sin(Th_1) * cos(phi));
C = (dz - L_1 -  L_4 * sin(phi));

% Calculate theta3 using the cosine rule
cos_Th3 = (A^2 + B^2 + C^2 - L_2^2 - L_3^2) / (2 * L_2 * L_3);
Th_3 = -acos(cos_Th3);

% Calculate intermediate variables a, b, and c
a_1 = L_3 * sin(Th_3);
b_1 = L_2 + L_3 * cos(Th_3);
c_1 = dz - L_1 - L_4 * sin(phi);
r_1 = sqrt(a_1^2 + b_1^2);

% Calculate theta2
Th_2 = atan2(c_1, sqrt(r_1^2 - c_1^2)) - atan2(a_1, b_1);

% Calculate theta4
Th_4 = phi - Th_2 - Th_3;

% Check if the calculated joint angles are within limits
if (Th_1 < Th1_min || Th_1 > Th1_max) || ...
   (Th_2 < Th2_min || Th_2 > Th2_max) || ...
   (Th_3 < Th3_min || Th_3 > Th3_max) || ...
   (Th_4 < Th4_min || Th_4 > Th4_max)
   
    % Display error message in the GUI's error text box
    set(handles.error_msg, 'String', ['Error: Input values are ' ...
        'out of the valid workspace range!'],'ForegroundColor',[1 0 0]);
    
    % Exit the function if there is an error
    return;
else
    % Clear the error message if angles are within range
    set(handles.error_msg, 'String', '');
end

%Simulation Plot
axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

handles.Theta_1.String = num2str(Th_1*180/pi);
handles.Theta_2.String = num2str(Th_2*180/pi);
handles.Theta_3.String = num2str(Th_3*180/pi);
handles.Theta_4.String = num2str(Th_4*180/pi);

%Servo Write Program for Inverse Push Button
clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes on button press in clear_all.
function clear_all_Callback(hObject, eventdata, handles)

%Home Position

set(handles.Theta_1,'string',num2str(0));
set(handles.Theta_2,'string',num2str(15));
set(handles.Theta_3,'string',num2str(-60));
set(handles.Theta_4,'string',num2str(-25));
set(handles.Pos_X,'string',num2str(0));
set(handles.Pos_Y,'string',num2str(0));
set(handles.Pos_Z,'string',num2str(0));
set(handles.Phi,'string',num2str(0));
set(handles.slider1,'value',0);
set(handles.slider2,'value',15);
set(handles.slider3,'value',-60);
set(handles.slider4,'value',-25);
set(handles.slider5,'value',0);
set(handles.slider6,'value',0);
set(handles.slider7,'value',0);
set(handles.slider8,'value',0);
cla(handles.axes2,'reset')

% Display error message in the GUI's error text box
set(handles.error_msg, 'String', '♡ ENJOY USING THIS MANIPULATOR ♡','ForegroundColor',[0 0 1]);

%Servo Write Program for Home Position

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,0.5);
pause(0.05)
writePosition(s2,1);
pause(0.05)
writePosition(s3,0);
pause(0.05)
writePosition(s4,0);
pause(0.05)

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)

Th_1 = get(handles.slider1,'value')*pi/180;
set(handles.Theta_1,'string',num2str(Th_1*180/pi));
Th_2 = get(handles.slider2,'value')*pi/180;
set(handles.Theta_2,'string',num2str(Th_2*180/pi));
Th_3 = get(handles.slider3,'value')*pi/180;
set(handles.Theta_3,'string',num2str(Th_3*180/pi));
Th_4 = get(handles.slider4,'value')*pi/180;
set(handles.Theta_4,'string',num2str(Th_4*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

axes(handles.axes2)
Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
M = T.T();
handles.Pos_X.String = num2str(floor(M(1,4)));
handles.Pos_Y.String = num2str(floor(M(2,4)));
handles.Pos_Z.String = num2str(floor(M(3,4)));
handles.Phi.String = num2str(floor((Th_2+Th_3+Th_4)*180/pi));

%Servo Write Program for Theta 1

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

function slider1_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)


Th_1 = get(handles.slider1,'value')*pi/180;
set(handles.Theta_1,'string',num2str(Th_1*180/pi));
Th_2 = get(handles.slider2,'value')*pi/180;
set(handles.Theta_2,'string',num2str(Th_2*180/pi));
Th_3 = get(handles.slider3,'value')*pi/180;
set(handles.Theta_3,'string',num2str(Th_3*180/pi));
Th_4 = get(handles.slider4,'value')*pi/180;
set(handles.Theta_4,'string',num2str(Th_4*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

axes(handles.axes2)
Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
M = T.T();
handles.Pos_X.String = num2str(floor(M(1,4)));
handles.Pos_Y.String = num2str(floor(M(2,4)));
handles.Pos_Z.String = num2str(floor(M(3,4)));
handles.Phi.String = num2str(floor((Th_2+Th_3+Th_4)*180/pi));

%Servo Write Program for Theta 2

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)

Th_1 = get(handles.slider1,'value')*pi/180;
set(handles.Theta_1,'string',num2str(Th_1*180/pi));
Th_2 = get(handles.slider2,'value')*pi/180;
set(handles.Theta_2,'string',num2str(Th_2*180/pi));
Th_3 = get(handles.slider3,'value')*pi/180;
set(handles.Theta_3,'string',num2str(Th_3*180/pi));
Th_4 = get(handles.slider4,'value')*pi/180;
set(handles.Theta_4,'string',num2str(Th_4*180/pi));


L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

axes(handles.axes2)
Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
M = T.T();
handles.Pos_X.String = num2str(floor(M(1,4)));
handles.Pos_Y.String = num2str(floor(M(2,4)));
handles.Pos_Z.String = num2str(floor(M(3,4)));
handles.Phi.String = num2str(floor((Th_2+Th_3+Th_4)*180/pi));

%Servo Write Program for Theta 3

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)

Th_1 = get(handles.slider1,'value')*pi/180;
set(handles.Theta_1,'string',num2str(Th_1*180/pi));
Th_2 = get(handles.slider2,'value')*pi/180;
set(handles.Theta_2,'string',num2str(Th_2*180/pi));
Th_3 = get(handles.slider3,'value')*pi/180;
set(handles.Theta_3,'string',num2str(Th_3*180/pi));
Th_4 = get(handles.slider4,'value')*pi/180;
set(handles.Theta_4,'string',num2str(Th_4*180/pi));


L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

axes(handles.axes2)
Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

T = Robot.fkine([Th_1 Th_2 Th_3 Th_4]);
M = T.T();
handles.Pos_X.String = num2str(floor(M(1,4)));
handles.Pos_Y.String = num2str(floor(M(2,4)));
handles.Pos_Z.String = num2str(floor(M(3,4)));
handles.Phi.String = num2str(floor((Th_2+Th_3+Th_4)*180/pi));

%Servo Write Program for Theta 4

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function text12_CreateFcn(hObject, eventdata, handles)

% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)

dx = get(handles.slider5,'value');
set(handles.Pos_X,'string',num2str(dx));
dy = get(handles.slider6,'value');
set(handles.Pos_Y,'string',num2str(dy));
dz = get(handles.slider7,'value');
set(handles.Pos_Z,'string',num2str(dz));
phi = get(handles.slider8,'value')*pi/180;
set(handles.Phi,'string',num2str(phi*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';

% Calculate theta1
Th_1 = atan2(dy, dx);

% Calculate a, b, c, and r
A = (dx - L_4 * cos(phi));
B = (dy - L_4 * sin(Th_1) * cos(phi));
C = (dz - L_1 -  L_4 * sin(phi));

% Calculate theta3 using the cosine rule
cos_Th3 = (A^2 + B^2 + C^2 - L_2^2 - L_3^2) / (2 * L_2 * L_3);
Th_3 = -acos(cos_Th3);

% Calculate intermediate variables a, b, and c
a_1 = L_3 * sin(Th_3);
b_1 = L_2 + L_3 * cos(Th_3);
c_1 = dz - L_1 - L_4 * sin(phi);
r_1 = sqrt(a_1^2 + b_1^2);

% Calculate theta2
Th_2 = atan2(c_1, sqrt(r_1^2 - c_1^2)) - atan2(a_1, b_1);

% Calculate theta4
Th_4 = phi - Th_2 - Th_3;

axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

handles.Theta_1.String = num2str(Th_1*180/pi);
handles.Theta_2.String = num2str(Th_2*180/pi);
handles.Theta_3.String = num2str(Th_3*180/pi);
handles.Theta_4.String = num2str(Th_4*180/pi);

%Servo Write Program for Px

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)

dx = get(handles.slider5,'value');
set(handles.Pos_X,'string',num2str(dx));
dy = get(handles.slider6,'value');
set(handles.Pos_Y,'string',num2str(dy));
dz = get(handles.slider7,'value');
set(handles.Pos_Z,'string',num2str(dz));
phi = get(handles.slider8,'value')*pi/180;
set(handles.Phi,'string',num2str(phi*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';

% Calculate theta1
Th_1 = atan2(dy, dx);

% Calculate a, b, c, and r
A = (dx - L_4 * cos(phi));
B = (dy - L_4 * sin(Th_1) * cos(phi));
C = (dz - L_1 -  L_4 * sin(phi));

% Calculate theta3 using the cosine rule
cos_Th3 = (A^2 + B^2 + C^2 - L_2^2 - L_3^2) / (2 * L_2 * L_3);
Th_3 = -acos(cos_Th3);

% Calculate intermediate variables a, b, and c
a_1 = L_3 * sin(Th_3);
b_1 = L_2 + L_3 * cos(Th_3);
c_1 = dz - L_1 - L_4 * sin(phi);
r_1 = sqrt(a_1^2 + b_1^2);

% Calculate theta2
Th_2 = atan2(c_1, sqrt(r_1^2 - c_1^2)) - atan2(a_1, b_1);

% Calculate theta4
Th_4 = phi - Th_2 - Th_3;

axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

handles.Theta_1.String = num2str(Th_1*180/pi);
handles.Theta_2.String = num2str(Th_2*180/pi);
handles.Theta_3.String = num2str(Th_3*180/pi);
handles.Theta_4.String = num2str(Th_4*180/pi);

%Servo Write Program for Py

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)

dx = get(handles.slider5,'value');
set(handles.Pos_X,'string',num2str(dx));
dy = get(handles.slider6,'value');
set(handles.Pos_Y,'string',num2str(dy));
dz = get(handles.slider7,'value');
set(handles.Pos_Z,'string',num2str(dz));
phi = get(handles.slider8,'value')*pi/180;
set(handles.Phi,'string',num2str(phi*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';

% Calculate theta1
Th_1 = atan2(dy, dx);

% Calculate a, b, c, and r
A = (dx - L_4 * cos(phi));
B = (dy - L_4 * sin(Th_1) * cos(phi));
C = (dz - L_1 -  L_4 * sin(phi));

% Calculate theta3 using the cosine rule
cos_Th3 = (A^2 + B^2 + C^2 - L_2^2 - L_3^2) / (2 * L_2 * L_3);
Th_3 = -acos(cos_Th3);

% Calculate intermediate variables a, b, and c
a_1 = L_3 * sin(Th_3);
b_1 = L_2 + L_3 * cos(Th_3);
c_1 = dz - L_1 - L_4 * sin(phi);
r_1 = sqrt(a_1^2 + b_1^2);

% Calculate theta2
Th_2 = atan2(c_1, sqrt(r_1^2 - c_1^2)) - atan2(a_1, b_1);

% Calculate theta4
Th_4 = phi - Th_2 - Th_3;

axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

handles.Theta_1.String = num2str(Th_1*180/pi);
handles.Theta_2.String = num2str(Th_2*180/pi);
handles.Theta_3.String = num2str(Th_3*180/pi);
handles.Theta_4.String = num2str(Th_4*180/pi);

%Servo Write Program for Pz

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)

dx = get(handles.slider5,'value');
set(handles.Pos_X,'string',num2str(dx));
dy = get(handles.slider6,'value');
set(handles.Pos_Y,'string',num2str(dy));
dz = get(handles.slider7,'value');
set(handles.Pos_Z,'string',num2str(dz));
phi = get(handles.slider8,'value')*pi/180;
set(handles.Phi,'string',num2str(phi*180/pi));

L_1 = 215;
L_2 = 213;
L_3 = 110;
L_4 =73;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);
L(4) = Link([0 0 L_4 0]);

Robot = SerialLink(L);
Robot.name = '4-DOF Manipulator';

% Calculate theta1
Th_1 = atan2(dy, dx);

% Calculate a, b, c, and r
A = (dx - L_4 * cos(phi));
B = (dy - L_4 * sin(Th_1) * cos(phi));
C = (dz - L_1 -  L_4 * sin(phi));

% Calculate theta3 using the cosine rule
cos_Th3 = (A^2 + B^2 + C^2 - L_2^2 - L_3^2) / (2 * L_2 * L_3);
Th_3 = -acos(cos_Th3);

% Calculate intermediate variables a, b, and c
a_1 = L_3 * sin(Th_3);
b_1 = L_2 + L_3 * cos(Th_3);
c_1 = dz - L_1 - L_4 * sin(phi);
r_1 = sqrt(a_1^2 + b_1^2);

% Calculate theta2
Th_2 = atan2(c_1, sqrt(r_1^2 - c_1^2)) - atan2(a_1, b_1);

% Calculate theta4
Th_4 = phi - Th_2 - Th_3;

axes(handles.axes2)
Robot.plot([Th_1 Th_2 Th_3 Th_4]);

handles.Theta_1.String = num2str(Th_1*180/pi);
handles.Theta_2.String = num2str(Th_2*180/pi);
handles.Theta_3.String = num2str(Th_3*180/pi);
handles.Theta_4.String = num2str(Th_4*180/pi);

%Servo Write Program for Phi

clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

writePosition(s1,((Th_1*(180/pi))+90)/200);
pause(0.05)
writePosition(s2,((Th_2*(180/pi))-15)/30);
pause(0.05)
writePosition(s3,((Th_3*(180/pi))+130)/70);
pause(0.05)
writePosition(s4,((Th_4*(180/pi))+90)/65);
pause(0.05)

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)

function axes3_CreateFcn(hObject, eventdata, handles)

function backgroundIMG_CreateFcn(hObject, eventdata, handles)
    
%Background Image
imshow('D:\MTULoGo1.jpg')

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)

%loop function of 3 position
clear global
a = arduino('COM21','Uno','Libraries','Servo');
s1 = servo(a,'D5');
s2 = servo(a,'D6');
s3 = servo(a,'D9');
s4 = servo(a,'D10');

k=0;
while k<3
    
    %Home Position

    writePosition(s1,(0+90)/200); %T1=0;
    pause(0.05)
    writePosition(s2,(45-15)/30); %T2=45
    pause(0.05)
    writePosition(s3,(-60+130)/70); %T3=-60
    pause(0.05)
    writePosition(s4,(-25+90)/65);%T4=-25
    pause(0.05)
    
    pause(2)

    %Position 1
    
    writePosition(s1,(90+90)/200); %T1=90;
    pause(0.05)
    writePosition(s2,(45-15)/30); %T2=45
    pause(0.05)
    writePosition(s3,(-130+130)/70); %T3=-130
    pause(0.05)
    writePosition(s4,(-90+90)/65);%T4=-90
    pause(0.05)
    
    pause(3)

    %Home Position

    writePosition(s1,(0+90)/200); %T1=0;
    pause(0.05)
    writePosition(s2,(45-15)/30); %T2=45
    pause(0.05)
    writePosition(s3,(-60+130)/70); %T3=-60
    pause(0.05)
    writePosition(s4,(-25+90)/65);%T4=-25
    pause(2)

    %Position 2
    
    writePosition(s1,(-90+90)/200); %T1=-90;
    pause(0.05)
    writePosition(s2,(15-15)/30); %T2=15
    pause(0.05)
    writePosition(s3,(-90+130)/70); %T3=-90
    pause(0.05)
    writePosition(s4,(-45+90)/65);%T4=-45
    pause(0.05)
    
    pause(2)
    
    k=k+1;
end



function error_msg_Callback(hObject, eventdata, handles)
% hObject    handle to error_msg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of error_msg as text
%        str2double(get(hObject,'String')) returns contents of error_msg as a double


% --- Executes during object creation, after setting all properties.
function error_msg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to error_msg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
