function varargout = trackBall(varargin)
% TRACKBALL MATLAB code for trackBall.fig
%      TRACKBALL, by itself, creates a new TRACKBALL or raises the existing
%      singleton*.
%
%      H = TRACKBALL returns the handle to a new TRACKBALL or the handle to
%      the existing singleton*.
%
%      TRACKBALL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKBALL.M with the given input arguments.
%
%      TRACKBALL('Property','Value',...) creates a new TRACKBALL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trackBall_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trackBall_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trackBall

% Last Modified by GUIDE v2.5 06-Jan-2018 13:57:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trackBall_OpeningFcn, ...
                   'gui_OutputFcn',  @trackBall_OutputFcn, ...
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


% --- Executes just before trackBall is made visible.
function trackBall_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trackBall (see VARARGIN)

set(hObject,'WindowButtonDownFcn',{@my_MouseClickFcn,handles.axes1});
set(hObject,'WindowButtonUpFcn',{@my_MouseReleaseFcn,handles.axes1});
axes(handles.axes1);

handles.Cube=DrawCube(eye(3));

set(handles.axes1,'CameraPosition',...
    [0 0 5],'CameraTarget',...
    [0 0 -5],'CameraUpVector',...
    [0 1 0],'DataAspectRatio',...
    [1 1 1]);

set(handles.axes1,'xlim',[-3 3],'ylim',[-3 3],'visible','off','color','none');

% Choose default command line output for trackBall
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trackBall wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = trackBall_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function my_MouseClickFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)
    Set_init_vec(GetRotVec(3,xmouse,ymouse));
    set(handles.figure1,'WindowButtonMotionFcn',{@my_MouseMoveFcn,hObject});
end
guidata(hObject,handles)

function my_MouseReleaseFcn(obj,event,hObject)
handles=guidata(hObject);
set(handles.figure1,'WindowButtonMotionFcn','');
guidata(hObject,handles);

function my_MouseMoveFcn(obj,event,hObject)

handles=guidata(obj);
xlim = get(handles.axes1,'xlim');
ylim = get(handles.axes1,'ylim');
mousepos=get(handles.axes1,'CurrentPoint');
xmouse = mousepos(1,1);
ymouse = mousepos(1,2);

if xmouse > xlim(1) && xmouse < xlim(2) && ymouse > ylim(1) && ymouse < ylim(2)
    
    init_vector = Get_init_vec();
    axis = -cross(GetRotVec(3,xmouse,ymouse), init_vector);
    angle = acosd((GetRotVec(3,xmouse,ymouse)'*init_vector)/(norm(GetRotVec(3,xmouse,ymouse))*norm(init_vector)))*0.2;
    
    r_mat = Eaa2rotMat(axis, angle);
    
    %Write quaternions
    if(isempty(GetLastQuaternion()))
        q0 = [0;0;0;0];
    else
        q0 = GetLastQuaternion();
    end
    set(handles.q0_0,'String',q0(1,1));
    set(handles.q0_1,'String',q0(2,1));
    set(handles.q0_2,'String',q0(3,1));
    set(handles.q0_3,'String',q0(4,1));

    q1 = GetQuatFrom2Vec(init_vector,GetRotVec(3,xmouse,ymouse));
    set(handles.q1_0,'String',q1(1,1));
    set(handles.q1_1,'String',q1(2,1));
    set(handles.q1_2,'String',q1(3,1));
    set(handles.q1_3,'String',q1(4,1));

    Set_last_quaternion(q1);
    
    qk = q_product(q0,q1);
    set(handles.qk_0,'String',qk(1,1));
    set(handles.qk_1,'String',qk(2,1));
    set(handles.qk_2,'String',qk(3,1));
    set(handles.qk_3,'String',qk(4,1));

    %Write Euler axis & angle
    [e_axis,e_angle]=rotMat2Eaa(r_mat);
    set(handles.e_axis_x,'String',e_axis(1,1));
    set(handles.e_axis_y,'String',e_axis(2,1));
    set(handles.e_axis_z,'String',e_axis(3,1));
    set(handles.e_angle,'String',e_angle);
   
    %Write Euler Angles
    [phi,theta,psi]=rotM2eAngles(r_mat);
    set(handles.phi,'String',phi);
    set(handles.theta,'String',theta);
    set(handles.psi,'String',psi);

    %Write rotation vector
    %[e_axis,e_angle]=rotMat2Eaa(r_mat);
    %v_rot = RotVec(init_vector,e_angle,e_axis);
    %set(handles.x,'String',v_rot(1));
    %set(handles.y,'String',v_rot(2));
    %set(handles.z,'String',v_rot(3));

    
    %Write r_mat 
    Set_rotation_matrix(r_mat);
    
    set(handles.r_m_1_1,'String',r_mat(1,1));
    set(handles.r_m_1_2,'String',r_mat(1,2));
    set(handles.r_m_1_3,'String',r_mat(1,3));

    set(handles.r_m_2_1,'String',r_mat(2,1));
    set(handles.r_m_2_2,'String',r_mat(2,2));
    set(handles.r_m_2_3,'String',r_mat(2,3));

    set(handles.r_m_3_1,'String',r_mat(3,1));
    set(handles.r_m_3_2,'String',r_mat(3,2));
    set(handles.r_m_3_3,'String',r_mat(3,3));
    
    
    
    
    
    
    handles.Cube = RedrawCube(r_mat,handles.Cube);    
end
guidata(hObject,handles);

function h = DrawCube(R)

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

h = fill3(x,y,z, 1:6);

for q = 1:length(c)
    h(q).FaceColor = c(q,:);
end

function h = RedrawCube(R,hin)

h = hin;
c = 1/255*[255 248 88;
    0 0 0;
    57 183 225;
    57 183 0;
    255 178 0;
    255 0 0];

M0 = [    -1  -1 1;   %Node 1
    -1   1 1;   %Node 2
    1   1 1;   %Node 3
    1  -1 1;   %Node 4
    -1  -1 -1;  %Node 5
    -1   1 -1;  %Node 6
    1   1 -1;  %Node 7
    1  -1 -1]; %Node 8

M = (R*M0')';


x = M(:,1);
y = M(:,2);
z = M(:,3);


con = [1 2 3 4;
    5 6 7 8;
    4 3 7 8;
    1 2 6 5;
    1 4 8 5;
    2 3 7 6]';

x = reshape(x(con(:)),[4,6]);
y = reshape(y(con(:)),[4,6]);
z = reshape(z(con(:)),[4,6]);

for q = 1:6
    h(q).Vertices = [x(:,q) y(:,q) z(:,q)];
    h(q).FaceColor = c(q,:);
end

function Set_init_vec(v)
global initial_vector;
initial_vector = v;

function Set_rotation_matrix(r_mat)
global rotation_mat;
rotation_mat = r_mat;

function Set_last_quaternion(l_q)
global last_quat;
last_quat = l_q;

function vec = Get_init_vec();
global initial_vector;
vec = initial_vector;

function mat = Get_rotation_matrix();
global rotation_mat;
mat = rotation_mat;

function quat = GetLastQuaternion();
global last_quat;
quat = last_quat;

function q0_0_Callback(hObject, eventdata, handles)
% hObject    handle to q0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_0 as text
%        str2double(get(hObject,'String')) returns contents of q0_0 as a double


% --- Executes during object creation, after setting all properties.
function q0_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q0_1_Callback(hObject, eventdata, handles)
% hObject    handle to q0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_1 as text
%        str2double(get(hObject,'String')) returns contents of q0_1 as a double


% --- Executes during object creation, after setting all properties.
function q0_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q0_2_Callback(hObject, eventdata, handles)
% hObject    handle to q0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_2 as text
%        str2double(get(hObject,'String')) returns contents of q0_2 as a double


% --- Executes during object creation, after setting all properties.
function q0_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q0_3_Callback(hObject, eventdata, handles)
% hObject    handle to q0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q0_3 as text
%        str2double(get(hObject,'String')) returns contents of q0_3 as a double


% --- Executes during object creation, after setting all properties.
function q0_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1_0_Callback(hObject, eventdata, handles)
% hObject    handle to q1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_0 as text
%        str2double(get(hObject,'String')) returns contents of q1_0 as a double


% --- Executes during object creation, after setting all properties.
function q1_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1_1_Callback(hObject, eventdata, handles)
% hObject    handle to q1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_1 as text
%        str2double(get(hObject,'String')) returns contents of q1_1 as a double


% --- Executes during object creation, after setting all properties.
function q1_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1_2_Callback(hObject, eventdata, handles)
% hObject    handle to q1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_2 as text
%        str2double(get(hObject,'String')) returns contents of q1_2 as a double


% --- Executes during object creation, after setting all properties.
function q1_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function q1_3_Callback(hObject, eventdata, handles)
% hObject    handle to q1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of q1_3 as text
%        str2double(get(hObject,'String')) returns contents of q1_3 as a double


% --- Executes during object creation, after setting all properties.
function q1_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function qk_0_Callback(hObject, eventdata, handles)
% hObject    handle to qk_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_0 as text
%        str2double(get(hObject,'String')) returns contents of qk_0 as a double

% --- Executes during object creation, after setting all properties.
function qk_0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function qk_1_Callback(hObject, eventdata, handles)
% hObject    handle to qk_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_1 as text
%        str2double(get(hObject,'String')) returns contents of qk_1 as a double

% --- Executes during object creation, after setting all properties.
function qk_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function qk_2_Callback(hObject, eventdata, handles)
% hObject    handle to qk_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_2 as text
%        str2double(get(hObject,'String')) returns contents of qk_2 as a double

% --- Executes during object creation, after setting all properties.
function qk_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function qk_3_Callback(hObject, eventdata, handles)
% hObject    handle to qk_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of qk_3 as text
%        str2double(get(hObject,'String')) returns contents of qk_3 as a double

% --- Executes during object creation, after setting all properties.
function qk_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qk_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function e_axis_x_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_axis_x as text
%        str2double(get(hObject,'String')) returns contents of e_axis_x as a double

% --- Executes during object creation, after setting all properties.
function e_axis_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function e_axis_y_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_axis_y as text
%        str2double(get(hObject,'String')) returns contents of e_axis_y as a double

% --- Executes during object creation, after setting all properties.
function e_axis_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function e_axis_z_Callback(hObject, eventdata, handles)
% hObject    handle to e_axis_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of e_axis_z as text
%        str2double(get(hObject,'String')) returns contents of e_axis_z as a double

% --- Executes during object creation, after setting all properties.
function e_axis_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_axis_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function e_angle_Callback(hObject, eventdata, handles)
% hObject    handle to e_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of e_angle as text
%        str2double(get(hObject,'String')) returns contents of e_angle as a double

% --- Executes during object creation, after setting all properties.
function e_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to e_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function phi_Callback(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of phi as text
%        str2double(get(hObject,'String')) returns contents of phi as a double

% --- Executes during object creation, after setting all properties.
function phi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to phi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function theta_Callback(hObject, eventdata, handles)
% hObject    handle to theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of theta as text
%        str2double(get(hObject,'String')) returns contents of theta as a double

% --- Executes during object creation, after setting all properties.
function theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function psi_Callback(hObject, eventdata, handles)
% hObject    handle to psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of psi as text
%        str2double(get(hObject,'String')) returns contents of psi as a double

% --- Executes during object creation, after setting all properties.
function psi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to psi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as a double

% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as a double

% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as a double

% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pusheulerangles.
function pusheulerangles_Callback(hObject, eventdata, handles)
% hObject    handle to pusheulerangles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
phi = str2double(get(handles.phi,'String'));
theta = str2double(get(handles.theta,'String'));
psi = str2double(get(handles.psi,'String'));

r_mat=eAngles2rotM(phi,theta,psi);
init_vector = Get_init_vec();
%---------------------------Do All Again------------------------
    %Write quaternions
    if(GetLastQuaternion()==0)
        q0 = [0;0;0;0];
    else
    q0 = GetLastQuaternion();
    end
    set(handles.q0_0,'String',q0(1,1));
    set(handles.q0_1,'String',q0(2,1));
    set(handles.q0_2,'String',q0(3,1));
    set(handles.q0_3,'String',q0(4,1));

    q1 = eul2quat(phi, theta, psi);
    set(handles.q1_0,'String',q1(1,1));
    set(handles.q1_1,'String',q1(2,1));
    set(handles.q1_2,'String',q1(3,1));
    set(handles.q1_3,'String',q1(4,1));

    Set_last_quaternion(q1);
    
    qk = q_product(q0,q1);
    set(handles.qk_0,'String',qk(1,1));
    set(handles.qk_1,'String',qk(2,1));
    set(handles.qk_2,'String',qk(3,1));
    set(handles.qk_3,'String',qk(4,1));

    %Write Euler axis & angle
    [e_axis,e_angle]=rotMat2Eaa(r_mat);
    set(handles.e_axis_x,'String',e_axis(1,1));
    set(handles.e_axis_y,'String',e_axis(2,1));
    set(handles.e_axis_z,'String',e_axis(3,1));
    set(handles.e_angle,'String',e_angle);
   
    %Write Euler Angles
    [phi,theta,psi]=rotM2eAngles(r_mat);
    set(handles.phi,'String',phi);
    set(handles.theta,'String',theta);
    set(handles.psi,'String',psi);

    %Write rotation vector
    [e_axis,e_angle]=rotMat2Eaa(r_mat);
    v_rot = RotVec(init_vector,e_angle,e_axis);
    set(handles.x,'String',v_rot(1));
    set(handles.y,'String',v_rot(2));
    set(handles.z,'String',v_rot(3));
    
    %Write r_mat 
    Set_rotation_matrix(r_mat);
    
    set(handles.r_m_1_1,'String',r_mat(1,1));
    set(handles.r_m_1_2,'String',r_mat(1,2));
    set(handles.r_m_1_3,'String',r_mat(1,3));

    set(handles.r_m_2_1,'String',r_mat(2,1));
    set(handles.r_m_2_2,'String',r_mat(2,2));
    set(handles.r_m_2_3,'String',r_mat(2,3));

    set(handles.r_m_3_1,'String',r_mat(3,1));
    set(handles.r_m_3_2,'String',r_mat(3,2));
    set(handles.r_m_3_3,'String',r_mat(3,3));
    
    handles.Cube = RedrawCube(r_mat,handles.Cube);    

% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.axes1,'CameraPosition',...
    [0 0 5],'CameraTarget',...
    [0 0 -5],'CameraUpVector',...
    [0 1 0],'DataAspectRatio',...
    [1 1 1]);

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1

% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pusheuleraxis.
function pusheuleraxis_Callback(hObject, eventdata, handles)
% hObject    handle to pusheuleraxis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushquaternions.
function pushquaternions_Callback(hObject, eventdata, handles)
% hObject    handle to pushquaternions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushrotvec.
function pushrotvec_Callback(hObject, eventdata, handles)
% hObject    handle to pushrotvec (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
