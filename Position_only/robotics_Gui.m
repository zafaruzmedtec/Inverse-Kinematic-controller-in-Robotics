function varargout = robotics_Gui(varargin)
% ROBOTICS_GUI MATLAB code for robotics_Gui.fig
%      ROBOTICS_GUI, by itself, creates a new ROBOTICS_GUI or raises the existing
%      singleton*.
%
%      H = ROBOTICS_GUI returns the handle to a new ROBOTICS_GUI or the handle to
%      the existing singleton*.
%
%      ROBOTICS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTICS_GUI.M with the given input arguments.
%
%      ROBOTICS_GUI('Property','Value',...) creates a new ROBOTICS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robotics_Gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robotics_Gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robotics_Gui

% Last Modified by GUIDE v2.5 01-Jul-2018 18:54:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robotics_Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @robotics_Gui_OutputFcn, ...
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


% --- Executes just before robotics_Gui is made visible.
function robotics_Gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robotics_Gui (see VARARGIN)

% Choose default command line output for robotics_Gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes robotics_Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = robotics_Gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
kuka_init 

%% Initialization

tfInput=str2num(handles.edit5.String);
TsInput=str2num(handles.edit6.String);

porta   = 19997;        % default V-REP port
n       = 7;            % number of joints
tf      = tfInput;            % final time
Ts      = TsInput;         % sampling time
%Ts      = 1e-2;         % sampling time
t       = 0:Ts:1.5*tf;  % time vector
N       = length(t);    % number of points of the simulation

%% Calculating Homogenious Function.
T               = DirectKinematics(DH);

% T has multiplication of consequative Homogeneous function
%T(4,4,1-7)
% T has the final Homogeneous function at position T(:,:,7).

%% Position of the EE.
%hk=get(handles.edit1,'String');
x=str2num(handles.edit3.String);
y=str2num(handles.twovalue.String);
z=str2num(handles.edit4.String);
ee_pos_initial  = [-0.55, -0.32,  0.31]'; %giving initial postiton
ee_pos_final    = [x,  y,  z]'; %giving the final position
%ee_pos_final    = [-0.55,  w,  0.25]'; %giving the final position
ee_quat_final   = Rot2Quat(T(1:3,1:3,n)); %converting Rotation to Quardient.
ee_vel_cruize   = [1 ,  1 ,  1 ]' .* abs(ee_pos_final - ee_pos_initial) / tf;

q       = zeros(n,N);           % q(:,i) collects the joint position for t(i)
qvrep   = zeros(n,N);
q_dot   = zeros(n,N);           % dq(:,i) collects the joint velocity for t(i)

pos_trpz        = zeros(3,N);
vel_trpz        = zeros(3,N);
acc_trpz        = zeros(3,N);
final_pos_err   = zeros(3,N);
pos_err         = zeros(3,N);
quat_err        = zeros(3,N);

q(:,1)      = DH(:,4);
qvrep(:,1)  = q(:,1);
qvrep(2,1)  = -qvrep(2,1);
qvrep(6,1)  = -qvrep(6,1);

%% 
%% Start of simulation
clc
fprintf('----------------------');
fprintf('\n Simulation started ');
fprintf('\n Trying to connect...\n');
[clientID, vrep ] = StartVrep(porta);

[returnCode]= vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
pause(2)

handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints

my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,1)); % first move to q0
q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
pause(2)



%% Main simulation loop
for i=1:N
    DH(:,4) = q(:,i);
    [pos_trpz(:,i), vel_trpz(:,i), acc_trpz(:,i)] = Trapezoidal(ee_pos_initial, ee_pos_final, ee_vel_cruize, tf, t(i));
    [q_dot(:,i), final_pos_err(:,i), pos_err(:,i), quat_err(:,i)] = InverseKinematics(ee_pos_final, pos_trpz(:,i), ee_quat_final,  vel_trpz(:,i), DH);
    
    if i<N
        q(:,i+1) = q(:,i) + Ts * q_dot(:,i);
        J = Jacobian(DH);
        vel_trpz(:,i) = J(1:3,:)*q_dot(:,i);
    end
    
    qvrep(:,i) = q(:,i);
    qvrep(2,i) = -qvrep(2,i);
    qvrep(6,i) = -qvrep(6,i);
     %% Vrap simulation
    my_set_joint_target_position(vrep, clientID, handle_joint, qvrep(:,i));
    q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep
end
DeleteVrep(clientID, vrep);
%% Plot graphs
%%%%%%%%%%%%%%%%%%%%plot%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Position Error
axes(handles.axes7)
plot(t,final_pos_err)
title('Position Error')
xlabel('Time(s)')
ylabel('Error(m)')
grid on
%subplot(212)
%% Origentation Error
axes(handles.axes8)
plot(t,quat_err)
title('Orientation Error')
xlabel('Time(s)')
ylabel('Error(rad)')
grid on


% figure
% subplot(211)

%% End Effector Postion
axes(handles.axes9)
plot(t,pos_trpz)
title('End Effector Position')
xlabel('Time(s)')
ylabel('Position(m)')
legend('x', 'y', 'z')
grid on

%% End effector Velocity
%subplot(212)
axes(handles.axes10)
plot(t,vel_trpz)
title('End Effector Velocity')
xlabel('Time(s)')
ylabel('Linear velocity(m/s)')
legend('x', 'y', 'z')
grid on


% figure
% subplot(211)
%% Joint Position
axes(handles.axes11)
plot(t,q)
grid on
title('Joint Positions')
xlabel('Time(s)')
ylabel('Position(rad)')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7')

%% Joint velocity
% subplot(212)
axes(handles.axes12)
plot(t,q_dot)
% grid on
title('Joint Velocities')
xlabel('Time(s)')
ylabel('Angular velocity(rad/s)')
legend('dq1', 'dq2', 'dq3', 'dq4', 'dq5', 'dq6', 'dq7')


% figure
% hold on
%% Robot
axes(handles.axes6)
DH(:,4) = q(:,1);
DrawRobot(DH);
DH(:,4) = q(:,N);
DrawRobot(DH);



% axes(handles.axes1)
% DH(:,4) = q(:,1);
% DrawRobot(DH);
% DH(:,4) = q(:,N);
% DrawRobot(DH);
% guidata(hobjects,handles);


% --- Executes during object creation, after setting all properties.
function axes6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes6


% --- Executes during object creation, after setting all properties.
function axes7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes7


% --- Executes during object creation, after setting all properties.
function axes8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes8


% --- Executes during object creation, after setting all properties.
function axes9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes9


% --- Executes during object creation, after setting all properties.
function axes10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes10


% --- Executes during object creation, after setting all properties.
function axes11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes11


% --- Executes during object creation, after setting all properties.
function axes12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes12



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
input=str2num(get(hObject,'String'));
if(isempty(input))
   set(hObject,'String','0')
end
% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function twovalue_Callback(hObject, eventdata, handles)
% hObject    handle to twovalue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Hints: get(hObject,'String') returns contents of twovalue as text
%        str2double(get(hObject,'String')) returns contents of twovalue as a double


% --- Executes during object creation, after setting all properties.
function twovalue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to twovalue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
I=imread('MAIA_LOGO.jpg');
imshow(I)
% Hint: place code in OpeningFcn to populate axes13


% --- Executes during object creation, after setting all properties.
function axes14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
I=imread('unicasbk.jpg');
imshow(I);
% Hint: place code in OpeningFcn to populate axes14



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
