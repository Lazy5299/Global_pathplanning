                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         function varargout = PathFinding(varargin)
%      用户界面
%      PATHFINDING MATLAB code for PathFinding.fig
%      PATHFINDING, by itself, creates a new PATHFINDING or raises the existing
%      singleton*.
%
%      H = PATHFINDING returns the handle to a new PATHFINDING or the handle to
%      the existing singleton*.
%
%      PATHFINDING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PATHFINDING.M with the given input
%      arguments.XDCF.
%
%      PATHFINDING('Property','Value',...) creates a new PATHFINDING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PathFinding_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PathFinding_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PathFinding

% Last Modified by GUIDE v2.5 11-Apr-2022 18:06:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PathFinding_OpeningFcn, ...
                   'gui_OutputFcn',  @PathFinding_OutputFcn, ...
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


% --- Executes just before PathFinding is made visible.
function PathFinding_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PathFinding (see VARARGIN)

% Choose default command line output for PathFinding
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
global startflag
startflag = 0; %全局变量startflag
% UIWAIT makes PathFinding wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PathFinding_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function row_Callback(hObject, eventdata, handles)
% hObject    handle to row (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of row as text
%        str2double(get(hObject,'String')) returns contents of row as a double


% --- Executes during object creation, after setting all properties.
function row_CreateFcn(hObject, eventdata, handles)
% hObject    handle to row (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function column_Callback(hObject, eventdata, handles)
% hObject    handle to column (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of column as text
%        str2double(get(hObject,'String')) returns contents of column as a double


% --- Executes during object creation, after setting all properties.
function column_CreateFcn(hObject, eventdata, handles)
% hObject    handle to column (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes on button press in startbutton.
function startbutton_Callback(hObject, eventdata, handles)
% hObject    handle to startbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global startflag mapmatrix mapdata phandles edittype SandEpoint maprisk mapfar row column maze mapspeed mapz mapG
if startflag
    startflag = 0;
    set(handles.startbutton,'string','start');
else
    cla reset;
    
    startflag = 1;
    edittype = 100;
    set(handles.startbutton,'string','stop');
    row = str2double(get(handles.row,'string'));
    column = str2double(get(handles.column,'string')); %获取编辑框中的行列值
    nr=str2double(get(handles.normalrisk,'string'));
    nc=str2double(get(handles.normalcost,'string'));
    shangebianchang=str2double(get(handles.lengthofperblock,'string'));
    normalspeed=str2double(get(handles.normalspeed,'string'));
    topnb=shangebianchang/normalspeed;    %每个平地方格消耗时间
    SandEpoint = [1,1;row-1,column-1];
    mapmatrix = ones(row,column);
    mapdata = nc*ones(row,column);
    maprisk = nr*ones(row,column);
    mapfar  =shangebianchang*ones(row,column);
    mapspeed =topnb*ones(row,column);
    maze = zeros(row,column);
    mapz = zeros(row,column);
    mapG = zeros(row,column);
    colormap('winter');
    phandles = pcolor(1:row+1,1:column+1,[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]);
    hold on; %保持已经画的图
    set(phandles,'ButtonDownFcn',{@mapClickCallback,handles});
end
%%%
function mapClickCallback(hObject, eventdata, handles) %点击地图选择不同地形 
global startflag mapmatrix mapdata mapfar maprisk phandles edittype SandEpoint maze  mapspeed mapz mapG
if startflag
    shangebianchang=str2double(get(handles.lengthofperblock,'string'));  %每个栅格边长
    hightofnormal=str2double(get(handles.hightofnormal,'string'));       %平台高度
    upladderhight=str2double(get(handles.hightofupladder,'string'));     %竖梯高度
    hightofsmallwall=str2double(get(handles.hightofsmallwall,'string')); %矮墙高度
    hightofwall=str2double(get(handles.hightofwall,'string'));           %墙高度
    breadthoflittleriver=str2double(get(handles.breadthoflittleriver,'string')); %小溪宽度
    
    normalspeed=str2double(get(handles.normalspeed,'string'));     %平地行进速度
    climbspeed=str2double(get(handles.upladderspeed,'string'));    %爬梯子速度
    swingspeed=str2double(get(handles.ladderspeed,'string'));      %摆荡速度
    anglespeed=str2double(get(handles.anglespeed,'string'));       %斜坡速度
    jumpspeed=str2double(get(handles.jumpspeedperm,'string'));     %跳高速度
    throwoverspeed=str2double(get(handles.throwoverspeed,'string'));   %翻越速度
    longjumpspeed=str2double(get(handles.longjumpspeed,'string'));    %跳远速度
    
    
    topnb=shangebianchang/normalspeed;    %每个平地方格消耗时间
    toplb=shangebianchang/swingspeed;     %摆荡时间
    topab=shangebianchang/anglespeed;     %斜坡时间
    tocu=upladderhight/climbspeed;        %爬梯子时间
    tojump=hightofsmallwall/jumpspeed;    %跳高时间
    tothrowover=hightofwall/throwoverspeed; %翻越时间
    tolongjump=breadthoflittleriver/longjumpspeed; %跳远时间
    
    
    
    
    axesHandle = get(hObject,'Parent');
    temp = get(axesHandle,'CurrentPoint');%获取最近一次点击的位置
    coordinates = floor(temp(1,1:2));
    switch edittype
        case 1     %平地
            mapmatrix(coordinates(2),coordinates(1)) =1; %白色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.normalcost,'string'))*shangebianchang; 
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.normalrisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = topnb;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = hightofnormal ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 2     %高墙
            mapmatrix(coordinates(2),coordinates(1)) =4; %黑色
            mapdata(coordinates(2),coordinates(1)) = inf;
            mapfar (coordinates(2),coordinates(1)) = inf;
            maprisk(coordinates(2),coordinates(1)) = inf;
            mapspeed(coordinates(2),coordinates(1)) = inf;
            maze(coordinates(2),coordinates(1)) = 1 ;
            mapz(coordinates(2),coordinates(1)) = inf;
            mapG(coordinates(2),coordinates(1)) = 1 ;
        case 3     %河
            mapmatrix(coordinates(2),coordinates(1)) =2; %蓝色
            mapdata(coordinates(2),coordinates(1)) = inf;
            mapfar (coordinates(2),coordinates(1)) = inf;
            maprisk(coordinates(2),coordinates(1)) = inf;
            mapspeed(coordinates(2),coordinates(1)) = inf;
            maze(coordinates(2),coordinates(1)) = 1 ;
            mapz(coordinates(2),coordinates(1)) = inf;
            mapG(coordinates(2),coordinates(1)) = 1 ;
        case 4     %斜面
            mapmatrix(coordinates(2),coordinates(1)) = 3; %品红
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.anglecost,'string'))*shangebianchang;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.anglerisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = topab;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 5     %云梯
            mapmatrix(coordinates(2),coordinates(1)) = 5; %红色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.laddercost,'string'))*shangebianchang;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.ladderrisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = toplb;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 6
            mapmatrix(SandEpoint(1,2),SandEpoint(1,1)) = 1;
            SandEpoint(1,:) = coordinates;
            mapmatrix(coordinates(2),coordinates(1)) = 10;
            maze(coordinates(2),coordinates(1)) = 2 ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 7
            mapmatrix(SandEpoint(2,2),SandEpoint(2,1)) = 1;
            SandEpoint(2,:) = coordinates;
            mapmatrix(coordinates(2),coordinates(1)) = 10;
            maze(coordinates(2),coordinates(1)) = 3 ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 8    %竖梯
            mapmatrix(coordinates(2),coordinates(1)) = 6; %黄色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.upladdercost,'string'))*upladderhight;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.upladderrisk,'string'))*upladderhight;
            mapspeed(coordinates(2),coordinates(1)) = tocu;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = upladderhight;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 9    %矮墙
            mapmatrix(coordinates(2),coordinates(1)) = 7; %LightSlateGray 
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costofjumpperm,'string'))*hightofsmallwall;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.jumpriskperm,'string'))*hightofsmallwall;
            mapspeed(coordinates(2),coordinates(1)) = tojump;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 10    %墙
            mapmatrix(coordinates(2),coordinates(1)) = 8; %DimGrey
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costofthrowover,'string'))*hightofwall;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.throwoverrisk,'string'))*hightofwall;
            mapspeed(coordinates(2),coordinates(1)) = tothrowover;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 11    %溪流
            mapmatrix(coordinates(2),coordinates(1)) = 9; %DeepSkyBlue
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costoflongjumpperm,'string'))*breadthoflittleriver;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.riskoflongjumpperm,'string'))*breadthoflittleriver;
            mapspeed(coordinates(2),coordinates(1)) = tolongjump;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0 ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        otherwise
            return;
    end
    row = str2double(get(handles.row,'string'));
    column = str2double(get(handles.column,'string'));
    map=[1 1 1;  %白
        0 0 1;   %蓝
        1 0 1;   %品红
        0 0 0;   %黑
        1 0 0;   %红
        1 1 0;   %黄
        0.46667 0.53333 0.6      %LightSlateGray 
        0.41176 0.41176 0.41176  %DimGrey
        0 0.74902 1              %DeepSkyBlue
        0 1 0];  %绿
    colormap(map);
    phandles = pcolor(1:row+1,1:column+1,[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]);
    set(phandles,'ButtonDownFcn',{@mapClickCallback,handles}); 
end


% --- Executes on button press in ladder.  梯子
function ladder_Callback(hObject, eventdata, handles)
% hObject    handle to ladder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ladder
global edittype 
if get(handles.ladder,'value')
    edittype = 5;
    set(handles.normalground,'value',0);  
    set(handles.wall,'value',0);
    set(handles.river,'value',0);
    set(handles.angle,'value',0);
    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
    set(handles.upladder,'value',0);
    set(handles.smallwall,'value',0);
    set(handles.tallwall,'value',0);
    set(handles.littleriver,'value',0);
end


function laddercost_Callback(hObject, eventdata, handles)
% hObject    handle to laddercost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of laddercost as text
%        str2double(get(hObject,'String')) returns contents of laddercost as a double
global laddercost
laddercost=str2double(get(handles.laddercost,'string'));


% --- Executes during object creation, after setting all properties.
function laddercost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to laddercost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in angle.坡
function angle_Callback(hObject, eventdata, handles)
% hObject    handle to angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of angle
global edittype 
if get(handles.angle,'value')
    edittype = 4;
   set(handles.upladder,'value',0);
   set(handles.ladder,'value',0);
   set(handles.normalground,'value',0);  
   set(handles.smallwall,'value',0);
   set(handles.wall,'value',0);
   set(handles.tallwall,'value',0);
   set(handles.littleriver,'value',0);
   set(handles.river,'value',0);
   set(handles.startpoint,'value',0);
   set(handles.endpoint,'value',0);
end

% --- Executes on button press in river.河
function river_Callback(hObject, eventdata, handles)
% hObject    handle to river (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of river
global edittype 
if get(handles.river,'value')
    edittype = 3;
    set(handles.upladder,'value',0);
    set(handles.ladder,'value',0);
    set(handles.angle,'value',0);
    set(handles.normalground,'value',0);  
    set(handles.smallwall,'value',0);
    set(handles.wall,'value',0);
    set(handles.tallwall,'value',0);
    set(handles.littleriver,'value',0);

    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
end


function anglecost_Callback(hObject, eventdata, handles)
% hObject    handle to anglecost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of anglecost as text
%        str2double(get(hObject,'String')) returns contents of anglecost as a double
global anglecost
anglecost=str2double(get(handles.anglecost,'string'));

% --- Executes during object creation, after setting all properties.
function anglecost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to anglecost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in wall.
function wall_Callback(hObject, eventdata, handles)
% hObject    handle to wall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of wall
global edittype 
if get(handles.wall,'value')
    edittype = 10;
    set(handles.upladder,'value',0);
    set(handles.ladder,'value',0);
    set(handles.angle,'value',0);
    set(handles.normalground,'value',0);  
    set(handles.smallwall,'value',0);
    set(handles.tallwall,'value',0);
    set(handles.littleriver,'value',0);
    set(handles.river,'value',0);
    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
end

% --- Executes on button press in normalground.
function normalground_Callback(hObject, eventdata, handles)
% hObject    handle to normalground (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of normalground
global edittype 
if get(handles.normalground,'value')
    edittype = 1;
set(handles.upladder,'value',0);
set(handles.ladder,'value',0);
set(handles.angle,'value',0); 
set(handles.smallwall,'value',0);
set(handles.wall,'value',0);
set(handles.tallwall,'value',0);
set(handles.littleriver,'value',0);
set(handles.river,'value',0);
set(handles.startpoint,'value',0);
set(handles.endpoint,'value',0);
end


% --- Executes on button press in startpoint.
function startpoint_Callback(hObject, eventdata, handles)
% hObject    handle to startpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of startpoint
global edittype 
if get(handles.startpoint,'value')
    edittype = 6;
set(handles.upladder,'value',0);
set(handles.ladder,'value',0);
set(handles.angle,'value',0);
set(handles.normalground,'value',0);  
set(handles.smallwall,'value',0);
set(handles.wall,'value',0);
set(handles.tallwall,'value',0);
set(handles.littleriver,'value',0);
set(handles.river,'value',0);

set(handles.endpoint,'value',0);
end

% --- Executes on button press in endpoint.
function endpoint_Callback(hObject, eventdata, handles)
% hObject    handle to endpoint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of endpoint
global edittype 
if get(handles.endpoint,'value')
    edittype = 7;
set(handles.upladder,'value',0);
set(handles.ladder,'value',0);
set(handles.angle,'value',0);
set(handles.normalground,'value',0);  
set(handles.smallwall,'value',0);
set(handles.wall,'value',0);
set(handles.tallwall,'value',0);
set(handles.littleriver,'value',0);
set(handles.river,'value',0);
set(handles.startpoint,'value',0);

end

%% 功耗

% --- Executes on button press in pathfind. 最小功耗
function pathfind_Callback(hObject, eventdata, handles)
% hObject    handle to pathfind (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%方格数及障碍物比例的设定
tic
global SandEpoint mapdata mapmatrix mapfar maprisk row column mapspeed mapz
n = str2double(get(handles.row,'string'));   % 产生一个n x n的方格，修改此值可以修改生成图片的方格数
wallpercent = 0;  % 这个变量代表生成的障碍物占总方格数的比例 ，如0.5 表示障碍物占总格数的50%
Environmental_Set=1; %这个参数用来选择是否随机生成障碍物，若设定为0，则使用上一次创建的环境信息，若设定为1，则重新随机生成障碍物
Reset_GS=1;   %这个参数用来选择是否重新设定起始点和终止点，若设定为1，开始重新设定起始点和终止点，同时需要将变量New_goalposind和New_startposind的值修改为你所选择的起始点和终止点的索引值，设为0则关闭

% New_startposind=(SandEpoint(1,1)-1)*n+SandEpoint(1,2);  
% New_goalposind=(SandEpoint(2,1)-1)*n+SandEpoint(2,2);   
sgpoint=find(mapmatrix==10);
New_startposind=sgpoint(1); 
New_goalposind=sgpoint(2); 

[field, startposind, goalposind, costchart, fieldpointers] =initializeField(n,wallpercent); %以上的代码封装成一个函数取名为initializeField,输出量为field, startposind, goalposind, costchart, fieldpointers；随机生成包含障碍物，起始点，终止点等信息的矩阵


% normalcost=str2double(get(handles.normalcost,'string'));
% laddercost=str2double(get(handles.laddercost,'string'));
% upladdercost=str2double(get(handles.upladdercost,'string'));
% anglecost=str2double(get(handles.anglecost,'string'));
% b=[normalcost laddercost anglecost upladdercost];
% maxdata= max(b);

mindata= min(min(mapdata));               
loc1=find(mapdata==inf);
data = mapdata;
data(loc1)=mindata;                %先替换cost数据中的inf，以便归一化进行

speed=mapspeed;

minrisk=min(min(maprisk));              
loc2=find(maprisk==inf);
risk=maprisk;
risk(loc2)=minrisk;                %先替换risk数据中的inf，以便归一化进行


% shangebianchang=str2double(get(handles.lengthofperblock,'string'));
% GYfar = mapfar / shangebianchang; 
GYdata = mapminmax(data,0,1); %归一化处理    mapminmax的数学公式为y = (ymax-ymin)*(x-xmin)/(xmax-xmin) + ymin   ymin=0,ymax=1,所以y=(x-xmin)/(xmax-xmin)
GYrisk = mapminmax(risk,0, 1); %归一化处理
GYspeed = mapminmax(speed,0,1); %归一化处理
GYdata(loc1)=inf;
GYrisk(loc2)=inf;


cw = str2double(get(handles.costweight,'string')); %权重
fw = str2double(get(handles.farweight,'string'));  %权重
rw = str2double(get(handles.riskweight,'string')); %权重

if cw==1
    field=mapdata;
elseif fw==1
    field=mapspeed;
elseif rw==1
    field=maprisk;
else
    field= (cw * GYdata)+(fw * GYspeed)+(rw *  GYrisk) ; 
end
% field= (cw * GYdata)+(fw *  GYspeed)+(rw *  GYrisk) ;
% field= (cw * GYdata)+(fw *  GYspeed)+(rw *  maprisk) ;


%重新设定起始点和终止点
if(Reset_GS==1)
[field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, startposind, goalposind, costchart, fieldpointers,New_startposind,New_goalposind);
end
% 路径规划中用到的一些矩阵的初始化
setOpen = [startposind];  %setOpen 矩阵是用来存放待选的子节点（也就是拓展出来的子节点）的矩阵，初始化为起始点的索引值，在这个待选的矩阵里面找到最优的值，再存放到矩阵setClosed中，初始化为空矩阵
setOpenCosts = [0];  %setOpenCosts 中存放拓展出来的最优值距离起始点的代价，初始化为0（因为第一个点就是起始点，起始点距离）
setOpenHeuristics = [Inf]; %setOpenHeuristics 中存放待选的子节点距离终止点的代价
setClosed = []; 
setClosedCosts = [];
movementdirections = {'R','L','D','U','LD','LU','RU','RD'};  %，movementdirections中存放着四个移动方向的代号



% 这个函数用来随机生成环境，障碍物，起点，终点
axishandle = createFigure(handles,field,costchart,startposind,goalposind);    %将随机生成的方格及障碍物的数据生成图像

%%

% 这个while循环是本程序的核心，利用循环进行迭代来寻找终止点
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen) %与; 判断终点在没在openlist中 与 openlist是否为空，当终点不在openlist中且openlist不是空的时候
    [temp, ii] = min(setOpenCosts + setOpenHeuristics);     %寻找拓展出来的最小值；函数的返回值ii是（setOpenCosts + setOpenHeuristics）中的最小值在矩阵中的行数，返回值 temp 是 （setOpenCosts + setOpenHeuristics）中最小值
    
    %不同权重下不同H的算法
    if cw==1 || fw==1 
     [costs,heuristics,posinds] = findFValue(setOpen(ii),setOpenCosts(ii),field,goalposind,'taxicab'); 
    else 
     [costs,heuristics,posinds] = findFValue(setOpen(ii),setOpenCosts(ii),field,goalposind,'euclidean'); %封装函数findFValue,第533行,这个函数的作用就是把输入的点作为父节点，然后进行拓展找到子节点，并且找到子节点的代价，并且把子节点距离终点的代价找到;函数的输出量中costs表示拓展的子节点到起始点的代价，heuristics表示拓展出来的点到终止点的距离大约是多少，posinds表示拓展出来的子节点
      % 这个函数的输入参数中setOpen(ii)表示拓展点，setOpenCosts(ii)表示当前拓展点的代价，field是之前生成的环境 信息，goalposind是终止点的索引值，euclidean 是拓展的方法（在后面会介绍）
    end
    
  setClosed = [setClosed; setOpen(ii)];     % 将找出来的拓展出来的点中代价最小的那个点串到矩阵setClosed 中 
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];    % 将拓展出来的点中代价最小的那个点的代价串到矩阵setClosedCosts 中
  
  % 从setOpen中删除刚才放到矩阵setClosed中的那个点
  %如果这个点位于矩阵的内部
  if (ii > 1 && ii < length(setOpen))
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
    
  %如果这个点位于矩阵第一行
  elseif (ii == 1)
    setOpen = setOpen(2:end);
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
    
  %如果这个点位于矩阵的最后一行
  else
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
  
 %%  
  % 把拓展出来的点中符合要求的点放到setOpen 矩阵（openlist）中，作为待选点
  for jj=1:length(posinds)
   
    if ~isinf(costs(jj))   % 判断该点（方格）处没有障碍物,判断该点不是障碍物
        
      % 判断一下该点是否 已经存在于setOpen 矩阵或者setClosed 矩阵中
      % 如果我们要处理的拓展点既不在setOpen 矩阵，也不在setClosed 矩阵中
      if ~max([setClosed; setOpen] == posinds(jj))
        fieldpointers(posinds(jj)) = movementdirections(jj);
        costchart(posinds(jj)) = costs(jj);
        setOpen = [setOpen; posinds(jj)];
        setOpenCosts = [setOpenCosts; costs(jj)];
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];%把没在list中的点放入openlist中
        
      % 如果我们要处理的拓展点已经在setOpen 矩阵中
      elseif max(setOpen == posinds(jj))
        I = find(setOpen == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setOpenCosts(I) > costs(jj)
          costchart(setOpen(I)) = costs(jj);
          setOpenCosts(I) = costs(jj);
          setOpenHeuristics(I) = heuristics(jj);
          fieldpointers(setOpen(I)) = movementdirections(jj);
        end
        % 如果我们要处理的拓展点已经在setClosed 矩阵中
      else
        I = find(setClosed == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setClosedCosts(I) > costs(jj)
          costchart(setClosed(I)) = costs(jj);
          setClosedCosts(I) = costs(jj);
          fieldpointers(setClosed(I)) = movementdirections(jj);
        end
      end
    end
  end
  
 %% 
  
  if isempty(setOpen) 
      break; 
  end

end

%%

%调用findWayBack函数进行路径回溯，并绘制出路径曲线
if max(ismember(setOpen,goalposind))
  disp('Solution found!');
  p = findWayBack(goalposind,fieldpointers); % 调用findWayBack函数进行路径回溯，将回溯结果放于矩阵P中
 
  row = str2double(get(handles.row,'string'));
  column = str2double(get(handles.column,'string'));
    map=[0 0 1;   %不动1
        1 1 1;    %2
        1 0 1;    %3
        1 0 0;   %不动4
        0 0 0;   %5
        0.46667 0.53333 0.6      %不动6
        1 1 0;                 %7
        0 0.74902 1    %9 
        0.41176 0.41176 0.41176   %8
        0 1 0];   %不动10
    colormap(map);
%   colormap('prism');
  pcolor(handles.mapaxes,(1:row+1),(1:column+1),[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]); %绘制地图信息
  
  plot(p(:,2)+0.5,p(:,1)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);                                           %用 plot函数绘制路径曲线
  hold on;
  
  
  %drawnow;
  %drawnow;
elseif isempty(setOpen)
  errordlg('没有可行路径','错误'); 
end
sum1 = 0; 
sum2 = 0;
sum3 = 1;
sum5 = 0;
for i = 1:length(p)
    sum1 = sum1 + mapdata(p(i,1),p(i,2));
    sum2 = sum2 + mapfar(p(i,1),p(i,2));
    sum5 = sum5 + mapspeed(p(i,1),p(i,2));
end
shangebianchang=str2double(get(handles.lengthofperblock,'string'));
% nob=sum2/shangebianchang; %路径方格数

for i = 1:length(p)
    sum3 = sum3 * (1-maprisk(p(i,1),p(i,2)));
end
    sum4=1-sum3;


str1 = ['路径能耗为: ' num2str(sum1)];
str2 = ['时间为: ' num2str(sum5)];
str3 = ['路径摔倒率为: ' num2str(sum4)];
set(handles.textshow1,'string',str1);
set(handles.textshow2,'string',str2);
set(handles.textshow3,'string',str3);
toc
time=toc;
set(handles.time,'string',time);

% %粗略分段路径
% upladderhight=str2double(get(handles.hightofupladder,'string'));     %竖梯高度
% hightofnormal=str2double(get(handles.hightofnormal,'string'));       %平台高度
% 
% 
% zhengxupath=flipud(p)
% numberofpathpoint=length(zhengxupath);
% 
% shuangzu=[];
% sizu=[];
% baidang=[];
% panpa=[];
% tiaogao=[];
% fanyue=[];
% tiaoyuan=[];
% for no=1:numberofpathpoint
%     lastno=no-1;
%     if mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==1     %p平地双足行走
%         shuangzu=[shuangzu;zhengxupath(no,:) mapz(zhengxupath(no,1),zhengxupath(no,2))];  
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==3  %斜坡四足爬行
%         sizu=[sizu;zhengxupath(no,:) mapz(zhengxupath(lastno,1),zhengxupath(lastno,2))];
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==5   %云梯摆荡
%         baidang=[baidang;zhengxupath(no,:) upladderhight];
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==6   %竖梯攀爬
%         if mapmatrix(zhengxupath(lastno,1),zhengxupath(lastno,2))==5  %竖梯前一个格是云梯
%         panpa=[panpa;zhengxupath(no,:) mapz(zhengxupath(no,1),zhengxupath(no,2))];  
%         else   %竖梯前一个格不是云梯
%         panpa=[panpa;zhengxupath(no,:) mapz(zhengxupath(lastno,1),zhengxupath(lastno,2))];
%         end
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==7   %矮墙跳跃
%         tiaogao=[tiaogao;zhengxupath(no,:) mapz(zhengxupath(lastno,1),zhengxupath(lastno,2))];
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==8    %中墙翻越
%         fanyue=[fanyue;zhengxupath(no,:) mapz(zhengxupath(no,1),zhengxupath(no,2))];
%     elseif mapmatrix(zhengxupath(no,1),zhengxupath(no,2))==9    %溪流跳远
%         tiaoyuan=[tiaoyuan;zhengxupath(no,:) mapz(zhengxupath(lastno,1),zhengxupath(lastno,2))];
%     end
% end
% xlswrite('双足行走.xlsx',shuangzu);
% shuangzu(:,[1,2])=shuangzu(:,[2,1])
% sizu(:,[1,2])=sizu(:,[2,1])
% baidang(:,[1,2])=baidang(:,[2,1])
% panpa(:,[1,2])=panpa(:,[2,1])
% tiaogao(:,[1,2])=tiaogao(:,[2,1])
% fanyue(:,[1,2])=fanyue(:,[2,1])
% tiaoyuan(:,[1,2])=tiaoyuan(:,[2,1])
% shuangzu
% sizu
% baidang
% panpa
% tiaogao
% fanyue
% tiaoyuan


% %所有路径点的种类
% pathpointdata=[];
% for no=1:numberofpathpoint
%     data=mapmatrix(zhengxupath(no,1),zhengxupath(no,2));
%     pathpointdata=[pathpointdata;data];
% end
% 
% 
% %路径段数
% numberofsection=0;
% noopathmione=numberofpathpoint-1;
% for no=2:noopathmione
%    pointdata=pathpointdata(no);
%     if pointdata~=pathpointdata(no+1)
%         numberofsection=numberofsection+1;
%     end
% end
% numberofsection
% 
% 
% 
% %分段输出路径
% segmentation=1;
% pathindex=[];
% shuzi=1;
% for pathno=2:numberofsection
%     for no=shuzi:numberofpathpoint-1   
%     pointdata=pathpointdata(no);
%     if pointdata==pathpointdata(no+1)
%        pathindex=[pathindex ; zhengxupath(no,:); zhengxupath(no+1,:)];
%     else
%        pathindex=[pathindex;zhengxupath(no,:)];
%        
%        %去除分段路径列表中重复元素，并改为正确顺序
%        [uniquepathindex,locationnumber]=unique(pathindex,'rows','first');
%        res=sortrows([locationnumber,uniquepathindex]);
%        new_pathindex=res(:,2:3)
% %        new_pathindex(:,[1,2])=new_pathindex(:,[2,1])
%        
%        filename=strcat('segmentation_',num2str(segmentation),'.xls');
%        save(['C:\Users\lazy\Desktop\10.19\分段路径\',filename],'new_pathindex');
%        shuzi=no+1;
% %        xlswrite(filename,pathindex);
%        segmentation=segmentation+1;
%        pathindex=[];
%     end
%     end
% end
  



%% 

%findWayBack函数用来进行路径回溯，这个函数的输入参数是终止点goalposind和矩阵fieldpointers，输出参数是P
function p = findWayBack(goalposind,fieldpointers)

    n = length(fieldpointers);  % 获取环境的长度也就是n
    posind = goalposind;
    [py,px] = ind2sub([n,n],posind); % 将索引值posind转换为坐标值 [py,px]
    p = [py px];
    %利用while循环进行回溯，当我们回溯到起始点的时候停止，也就是在矩阵fieldpointers中找到S时停止
    while ~strcmp(fieldpointers{posind},'S')
      switch fieldpointers{posind}
        case 'L' % ’L’ 表示当前的点是由左边的点拓展出来的
          px = px - 1;
        case 'R' % ’R’ 表示当前的点是由右边的点拓展出来的
          px = px + 1;
        case 'U' % ’U’ 表示当前的点是由上面的点拓展出来的
          py = py - 1;
        case 'D' % ’D’ 表示当前的点是由下边的点拓展出来的
          py = py + 1;
        case 'RU'   %  表示当前的点是由右上边的点拓展出来的
          py = py - 1; 
          px = px + 1;
        case 'RD'   %  表示当前的点是由右下边的点拓展出来的
          py = py + 1; 
          px = px + 1;
        case 'LD'   %  表示当前的点是由左下边的点拓展出来的
          py = py + 1;
          px = px - 1;
        case 'LU'   %  表示当前的点是由左上边的点拓展出来的
          py = py - 1; 
          px = px - 1;
      end
      p = [p; py px];
      posind = sub2ind([n n],py,px);% 将坐标值转换为索引值
    end
%% 
%这个函数的作用就是把输入的点作为父节点，然后进行拓展找到子节点，并且找到子节点的代价，并且把子节点距离终点的代价找到。
%函数的输出量中costs表示拓展的子节点到起始点的代价，heuristics表示拓展出来的点到终止点的距离大约是多少，posinds表示拓展出来的子节点
function [cost,heuristic,posinds] = findFValue(posind,costsofar,field,goalind,heuristicmethod)
global sxy    
    n = length(field);  % 获取矩阵的长度
    [currentpos(1),currentpos(2)] = ind2sub([n n],posind);   %将要进行拓展的点（也就是父节点）的 索引值 拓展成 坐标值
    [goalpos(1),goalpos(2)] = ind2sub([n n],goalind);        %将终止点的 索引值 拓展成 坐标值
    [sx,sy] = ind2sub([n n],sxy); 
    cost = Inf*ones(8,1);
    heuristic = Inf*ones(8,1); 
    pos = ones(8,2); %将矩阵cost和heuristic初始化为4x1的无穷大值的矩阵，pos初始化为4x2的值为1的矩阵，为什么是4x1的呢，因为我们拓展的时候采用的是向上下左右四个方向进行拓展，也就是最多可以找到四个子节点，所以代价矩阵是4x1的，为啥是4x2的呢，因为pos中存放找到的子节点的坐标值，一个子节点需要两个位置，分别存放横纵坐标
    spsbn = abs(currentpos(2)-sy) + abs(currentpos(1)-sx);
    pgsbn = abs(currentpos(2)-goalpos(2)) + abs(currentpos(1)-goalpos(1));
    bn = spsbn + pgsbn ;

    
    % 拓展方向一
    newx = currentpos(2) - 1; newy = currentpos(1); 
    if newx > 0      % 判断拓展的点是否在区域内
      pos(1,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(1) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(1) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(1) = costsofar + field(newy,newx);

    end

    % 拓展方向二
    newx = currentpos(2) + 1; newy = currentpos(1);
    if newx <= n
      pos(2,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(2) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(2) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(2) = costsofar + field(newy,newx);

    end

    % 拓展方向三
    newx = currentpos(2); newy = currentpos(1)-1;
    if newy > 0
      pos(3,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(3) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(3) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(3) = costsofar + field(newy,newx);

    end

    % 拓展方向四
    newx = currentpos(2); newy = currentpos(1)+1;
    if newy <= n
      pos(4,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(4) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(4) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(4) = costsofar + field(newy,newx);
    
    end
    
    % 拓展方向五
    newx = currentpos(2)+1; newy = currentpos(1)-1;
    if newx <= n   &&  newy > 0
      pos(5,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(5) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(5) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(5) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向六
    newx = currentpos(2)+1; newy = currentpos(1)+1;
    if newx <= n && newy <= n
      pos(6,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(6) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(6) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(6) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向七
    newx = currentpos(2)-1; newy = currentpos(1)+1;
    if newx > 0 && newy <= n
      pos(7,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(7) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(7) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;
      end
      cost(7) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向八
    newx = currentpos(2)-1; newy = currentpos(1)-1;
    if newx > 0 && newy > 0
      pos(8,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(8) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(8) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(8) = costsofar + sqrt(2)*field(newy,newx);

    end
    posinds = sub2ind([n n],pos(:,1),pos(:,2)); % 将拓展出来的子节点的 坐标值 转换为 索引值
    
    

%% 
%这个矩阵的作用就是随机生成环境，障碍物，起始点，终止点等
function [field, startposind, goalposind, costchart, fieldpointers] =initializeField(n,wallpercent)
    field = ones(n,n) + 10*rand(n,n);%生成一个n*n的单位矩阵+0到10范围内的一个随机数
    field(ind2sub([n n],ceil(n^2.*rand(n*n*wallpercent,1)))) = Inf;%向上取整
    % 随机生成起始点和终止点
    startposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  %随机生成起始点的索引值
    goalposind = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %随机生成终止点的索引值
    field(startposind) = 0; field(goalposind) = 0;  %把矩阵中起始点和终止点处的值设为0
    
    costchart = NaN*ones(n,n);%生成一个nxn的矩阵costchart，每个元素都设为NaN。就是矩阵初始NaN无效数据
    costchart(startposind) = 0;%在矩阵costchart中将起始点位置处的值设为0
    
    % 生成元胞数组
    fieldpointers = cell(n,n);%生成元胞数组n*n
    fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G'; %将元胞数组的起始点的位置处设为 'S'，终止点处设为'G'
    fieldpointers(field==inf)={0};


%%
%利用随机生成的环境数据来进行环境的绘制
function axishandle = createFigure(handles,field,costchart,startposind,goalposind)
      
      n = length(field);  %获取矩阵的长度，并赋值给变量n
      field(field < Inf) = 0; %将fieid矩阵中的随机数（也就是没有障碍物的位置处）设为0
      pcolor(1:n+1,1:n+1,[field field(:,end); field(end,:) field(end,end)]);%多加了一个重复的（由n X n变为 n+1 X n+1 ）
 
      cmap = flipud(colormap('jet'));  %生成的cmap是一个256X3的矩阵，每一行的3个值都为0-1之间数，分别代表颜色组成的rgb值
      cmap(1,:) = zeros(3,1); cmap(end,:) = ones(3,1); %将矩阵cmap的第一行设为0 ，最后一行设为1
      colormap(flipud(cmap)); %进行颜色的倒转 
      hold on;
      axishandle = pcolor(handles.mapaxes,[1:n+1],[1:n+1],[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);  %将矩阵costchart进行拓展，插值着色后赋给axishandle
      hold on;
      
      [goalposy,goalposx] = ind2sub([n,n],goalposind);
      [startposy,startposx] = ind2sub([n,n],startposind);
       plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);
       hold on;
       plot(startposx+0.5,startposy+0.5,'go','MarkerSize',10,'LineWidth',6);
       hold on;
       
       
function [field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, startposind, goalposind, costchart, fieldpointers,New_startposind,New_goalposind)
    global sxy gxy
   %在initializeField函数创建环境时，将field矩阵中没有障碍物的点处设定为10，有障碍物的点处设定为inf，起始点和终止点设为0
   %现在需要将新的起始点和终止点处设为0，原来起始点和终止点处设为10
    field(startposind) = 10; field(goalposind) = 10;  
    field(New_startposind) = 0; field(New_goalposind) = 0;  
   
    %在initializeField函数创建环境时，将costchart矩阵中的起始点设定为0，其他点设定为NAN
    %现在需要将新的起始点设定为0，原来的起始点设定为1
    costchart(startposind) = NaN;
    costchart(New_startposind) =0;
    
    % 在initializeField函数创建环境时，将fieldpointers元胞数组中起始点的位置处设为'S'，终止点处设为'G'，有障碍物的点设为'0'，没有障碍物的点设为'1'
    % 现在需要将新的起始点处设为'S'，新的终止点处设为'G'，原来的起始点和终止点处设为'1'
    fieldpointers{startposind} = '1'; fieldpointers{goalposind} = '1'; 
    fieldpointers{New_startposind} = 'S'; fieldpointers{New_goalposind} = 'G'; 
  
    %最后我们将新的起始点和新的终止点的索引值分别赋值给startposind和goalposind进行输出
    startposind=New_startposind;
    sxy=New_startposind;
    goalposind=New_goalposind;
    gxy=New_goalposind;



function anglerisk_Callback(hObject, eventdata, handles)
% hObject    handle to anglerisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of anglerisk as text
%        str2double(get(hObject,'String')) returns contents of anglerisk as a double


% --- Executes during object creation, after setting all properties.
function anglerisk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to anglerisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called 

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ladderrisk_Callback(hObject, eventdata, handles)
% hObject    handle to ladderrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ladderrisk as text
%        str2double(get(hObject,'String')) returns contents of ladderrisk as a double


% --- Executes during object creation, after setting all properties.
function ladderrisk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ladderrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function normalrisk_Callback(hObject, eventdata, handles)  %normalrisk编辑框
% hObject    handle to normalrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of normalrisk as text
%        str2double(get(hObject,'String')) returns contents of normalrisk as a double


% --- Executes during object creation, after setting all properties.
function normalrisk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to normalrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function normalcost_Callback(hObject, eventdata, handles)       %normalcost编辑框
% hObject    handle to normalcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of normalcost as text
%        str2double(get(hObject,'String')) returns contents of normalcost as a double


% --- Executes during object creation, after setting all properties.
function normalcost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to normalcost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lengthofperblock_Callback(hObject, eventdata, handles)   %栅格边长编辑框
% hObject    handle to lengthofperblock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lengthofperblock as text
%        str2double(get(hObject,'String')) returns contents of lengthofperblock as a double


% --- Executes during object creation, after setting all properties.
function lengthofperblock_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lengthofperblock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function mapaxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mapaxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate mapaxes


% --- Executes on button press in refresh.
function refresh_Callback(hObject, eventdata, handles)
global startflag
% hObject    handle to refresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
startflag = 0;
str1 = [ ];
str2 = [ ];
str3 = [ ];
time = [ ];
set(handles.textshow1,'string',str1);
set(handles.textshow2,'string',str2);
set(handles.textshow3,'string',str3);
set(handles.time,'string',time);
cla reset;



% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ax = gca;
b= datetime;
exportgraphics(gca,'map.png','Resolution',300);



function riskweight_Callback(hObject, eventdata, handles)
% hObject    handle to riskweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of riskweight as text
%        str2double(get(hObject,'String')) returns contents of riskweight as a double


% --- Executes during object creation, after setting all properties.
function riskweight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to riskweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function farweight_Callback(hObject, eventdata, handles)
% hObject    handle to farweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of farweight as text
%        str2double(get(hObject,'String')) returns contents of farweight as a double


% --- Executes during object creation, after setting all properties.
function farweight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to farweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function costweight_Callback(hObject, eventdata, handles)
% hObject    handle to costweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of costweight as text
%        str2double(get(hObject,'String')) returns contents of costweight as a double


% --- Executes during object creation, after setting all properties.
function costweight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to costweight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in upladder.
function upladder_Callback(hObject, eventdata, handles)
% hObject    handle to upladder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of upladder
global edittype 
if get(handles.upladder,'value')
    edittype = 8;
    set(handles.normalground,'value',0);  
    set(handles.wall,'value',0);
    set(handles.river,'value',0);
    set(handles.angle,'value',0);
    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
    set(handles.ladder,'value',0);
    set(handles.smallwall,'value',0);
    set(handles.littleriver,'value',0);
    set(handles.tallwall,'value',0);
end



function upladderrisk_Callback(hObject, eventdata, handles)
% hObject    handle to upladderrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upladderrisk as text
%        str2double(get(hObject,'String')) returns contents of upladderrisk as a double


% --- Executes during object creation, after setting all properties.
function upladderrisk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upladderrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function upladdercost_Callback(hObject, eventdata, handles)
% hObject    handle to upladdercost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upladdercost as text
%        str2double(get(hObject,'String')) returns contents of upladdercost as a double


% --- Executes during object creation, after setting all properties.
function upladdercost_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upladdercost (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in astarpathfind.
function astarpathfind_Callback(hObject, eventdata, handles)
% hObject    handle to astarpathfind (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%方格数及障碍物比例的设定
tic
global SandEpoint mapdata mapmatrix mapfar maprisk row column Ablocknumber mapspeed
n = str2double(get(handles.row,'string'));   % 产生一个n x n的方格，修改此值可以修改生成图片的方格数
wallpercent = 0;  % 这个变量代表生成的障碍物占总方格数的比例 ，如0.5 表示障碍物占总格数的50%
Environmental_Set=1; %这个参数用来选择是否随机生成障碍物，若设定为0，则使用上一次创建的环境信息，若设定为1，则重新随机生成障碍物
Reset_GS=1;   %这个参数用来选择是否重新设定起始点和终止点，若设定为1，开始重新设定起始点和终止点，同时需要将变量New_goalposind和New_startposind的值修改为你所选择的起始点和终止点的索引值，设为0则关闭

% New_startposind=(SandEpoint(1,1)-1)*n+SandEpoint(1,2);  
% New_goalposind=(SandEpoint(2,1)-1)*n+SandEpoint(2,2);   
sgpoint=find(mapmatrix==10);
New_startposind=sgpoint(1); 
New_goalposind=sgpoint(2); 

[field, startposind, goalposind, costchart, fieldpointers] =initializeField(n,wallpercent); %以上的代码封装成一个函数取名为initializeField,输出量为field, startposind, goalposind, costchart, fieldpointers；随机生成包含障碍物，起始点，终止点等信息的矩阵


field=mapfar;


%重新设定起始点和终止点
if(Reset_GS==1)
[field, startposind, goalposind, costchart, fieldpointers] = Reset_G_S(field, startposind, goalposind, costchart, fieldpointers,New_startposind,New_goalposind);
end
% 路径规划中用到的一些矩阵的初始化
setOpen = [startposind];  %setOpen 矩阵是用来存放待选的子节点（也就是拓展出来的子节点）的矩阵，初始化为起始点的索引值，在这个待选的矩阵里面找到最优的值，再存放到矩阵setClosed中，初始化为空矩阵
setOpenCosts = [0];  %setOpenCosts 中存放拓展出来的最优值距离起始点的代价，初始化为0（因为第一个点就是起始点，起始点距离）
setOpenHeuristics = [Inf]; %setOpenHeuristics 中存放待选的子节点距离终止点的代价
setClosed = []; 
setClosedCosts = [];
movementdirections = {'R','L','D','U','LD','LU','RU','RD'};  %，movementdirections中存放着四个移动方向的代号



% 这个函数用来随机生成环境，障碍物，起点，终点
axishandle = createFigure(handles,field,costchart,startposind,goalposind);    %将随机生成的方格及障碍物的数据生成图像

%%

% 这个while循环是本程序的核心，利用循环进行迭代来寻找终止点
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen) %与; 判断终点在没在openlist中 与 openlist是否为空，当终点不在openlist中且openlist不是空的时候
    [temp, ii] = min(setOpenCosts + setOpenHeuristics);     %寻找拓展出来的最小值；函数的返回值ii是（setOpenCosts + setOpenHeuristics）中的最小值在矩阵中的行数，返回值 temp 是 （setOpenCosts + setOpenHeuristics）中最小值
    
    [costs,heuristics,posinds] = findFValueastar(setOpen(ii),setOpenCosts(ii),field,goalposind,'taxicab'); %封装函数findFValue,第533行,这个函数的作用就是把输入的点作为父节点，然后进行拓展找到子节点，并且找到子节点的代价，并且把子节点距离终点的代价找到;函数的输出量中costs表示拓展的子节点到起始点的代价，heuristics表示拓展出来的点到终止点的距离大约是多少，posinds表示拓展出来的子节点
                         % 这个函数的输入参数中setOpen(ii)表示拓展点，setOpenCosts(ii)表示当前拓展点的代价，field是之前生成的环境 信息，goalposind是终止点的索引值，euclidean 是拓展的方法（在后面会介绍）
    
    
  setClosed = [setClosed; setOpen(ii)];     % 将找出来的拓展出来的点中代价最小的那个点串到矩阵setClosed 中 
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];    % 将拓展出来的点中代价最小的那个点的代价串到矩阵setClosedCosts 中
  
  % 从setOpen中删除刚才放到矩阵setClosed中的那个点
  %如果这个点位于矩阵的内部
  if (ii > 1 && ii < length(setOpen))
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
    
  %如果这个点位于矩阵第一行
  elseif (ii == 1)
    setOpen = setOpen(2:end);
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
    
  %如果这个点位于矩阵的最后一行
  else
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
  
 %%  
  % 把拓展出来的点中符合要求的点放到setOpen 矩阵（openlist）中，作为待选点
  for jj=1:length(posinds)
   
    if ~isinf(costs(jj))   % 判断该点（方格）处没有障碍物,判断该点不是障碍物
        
      % 判断一下该点是否 已经存在于setOpen 矩阵或者setClosed 矩阵中
      % 如果我们要处理的拓展点既不在setOpen 矩阵，也不在setClosed 矩阵中
      if ~max([setClosed; setOpen] == posinds(jj))
        fieldpointers(posinds(jj)) = movementdirections(jj);
        costchart(posinds(jj)) = costs(jj);
        setOpen = [setOpen; posinds(jj)];
        setOpenCosts = [setOpenCosts; costs(jj)];
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];%把没在list中的点放入openlist中
        
      % 如果我们要处理的拓展点已经在setOpen 矩阵中
      elseif max(setOpen == posinds(jj))
        I = find(setOpen == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setOpenCosts(I) > costs(jj)
          costchart(setOpen(I)) = costs(jj);
          setOpenCosts(I) = costs(jj);
          setOpenHeuristics(I) = heuristics(jj);
          fieldpointers(setOpen(I)) = movementdirections(jj);
        end
        % 如果我们要处理的拓展点已经在setClosed 矩阵中
      else
        I = find(setClosed == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setClosedCosts(I) > costs(jj)
          costchart(setClosed(I)) = costs(jj);
          setClosedCosts(I) = costs(jj);
          fieldpointers(setClosed(I)) = movementdirections(jj);
        end
      end
    end
  end
  
 %% 
  
  if isempty(setOpen) 
      break; 
  end
%   set(axishandle,'CData',[costchart costchart(:,end); costchart(end,:) costchart(end,end)]);
%   set(gca,'CLim',[0 1.1*max(costchart(find(costchart < Inf)))]);
%   drawnow; 
end

%%

%调用findWayBack函数进行路径回溯，并绘制出路径曲线
if max(ismember(setOpen,goalposind))
  disp('Solution found!');
  p = findWayBack(goalposind,fieldpointers); % 调用findWayBack函数进行路径回溯，将回溯结果放于矩阵P中

  row = str2double(get(handles.row,'string'));
  column = str2double(get(handles.column,'string'));
    map=[0 0 1;   %不动1
        1 1 1;    %2
        1 0 1;    %3
        1 0 0;   %不动4
        0 0 0;   %5
        0.46667 0.53333 0.6      %不动6
        1 1 0;                 %7
        0 0.74902 1    %9 
        0.41176 0.41176 0.41176   %8
        0 1 0];   %不动10
    colormap(map);
%   colormap('prism');
  pcolor(handles.mapaxes,(1:row+1),(1:column+1),[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]); %绘制地图信息
  
  plot(p(:,2)+0.5,p(:,1)+0.5,'Color',0.2*ones(3,1),'LineWidth',4);                                           %用 plot函数绘制路径曲线
  hold on;
  
  
  %drawnow;
  %drawnow;
elseif isempty(setOpen)
  errordlg('没有可行路径','错误'); 
end
sum1 = 0; 
sum2 = 0;
sum3 = 1;
sum5 = 0;
for i = 1:length(p)
    sum1 = sum1 + mapdata(p(i,1),p(i,2));
    sum2 = sum2 + mapfar(p(i,1),p(i,2));

    sum5 = sum5 + mapspeed(p(i,1),p(i,2));
end
shangebianchang=str2double(get(handles.lengthofperblock,'string'));


for i = 1:length(p)
    sum3 = sum3 * (1-maprisk(p(i,1),p(i,2)));
end
    sum4=1-sum3;


str1 = ['路径能耗为: ' num2str(sum1)];
str2 = ['时间为: ' num2str(sum5)];
str3 = ['路径摔倒率为: ' num2str(sum4)];
set(handles.textshow1,'string',str1);
set(handles.textshow2,'string',str2);
set(handles.textshow3,'string',str3);
Ablocknumber = sum2;
toc
time=toc;
set(handles.time,'string',time);



function [cost,heuristic,posinds] = findFValueastar(posind,costsofar,field,goalind,heuristicmethod)
    n = length(field);  % 获取矩阵的长度
    [currentpos(1),currentpos(2)] = ind2sub([n n],posind);   %将要进行拓展的点（也就是父节点）的 索引值 拓展成 坐标值
    [goalpos(1),goalpos(2)] = ind2sub([n n],goalind);        %将终止点的 索引值 拓展成 坐标值
    cost = Inf*ones(8,1);
    heuristic = Inf*ones(8,1); 
    pos = ones(8,2); %将矩阵cost和heuristic初始化为4x1的无穷大值的矩阵，pos初始化为4x2的值为1的矩阵，为什么是4x1的呢，因为我们拓展的时候采用的是向上下左右四个方向进行拓展，也就是最多可以找到四个子节点，所以代价矩阵是4x1的，为啥是4x2的呢，因为pos中存放找到的子节点的坐标值，一个子节点需要两个位置，分别存放横纵坐标

    % 拓展方向一
    newx = currentpos(2) - 1; newy = currentpos(1); 
    if newx > 0      % 判断拓展的点是否在区域内
      pos(1,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy) ;

        case 'taxicab'
          heuristic(1) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy) ;

      end
      cost(1) = costsofar + field(newy,newx);

    end

    % 拓展方向二
    newx = currentpos(2) + 1; newy = currentpos(1);
    if newx <= n
      pos(2,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

        case 'taxicab'
          heuristic(2) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

      end
      cost(2) = costsofar + field(newy,newx);

    end

    % 拓展方向三
    newx = currentpos(2); newy = currentpos(1)-1;
    if newy > 0
      pos(3,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

        case 'taxicab'
          heuristic(3) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

      end
      cost(3) = costsofar + field(newy,newx);

    end

    % 拓展方向四
    newx = currentpos(2); newy = currentpos(1)+1;
    if newy <= n
      pos(4,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

        case 'taxicab'
          heuristic(4) = abs(goalpos(2)-newx) + abs(goalpos(1)-newy);

      end
      cost(4) = costsofar + field(newy,newx);

    end
    % 拓展方向五
    newx = currentpos(2)+1; newy = currentpos(1)-1;
    if newx <= n   &&  newy > 0
      pos(5,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(5) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(5) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(5) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向六
    newx = currentpos(2)+1; newy = currentpos(1)+1;
    if newx <= n && newy <= n
      pos(6,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(6) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(6) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(6) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向七
    newx = currentpos(2)-1; newy = currentpos(1)+1;
    if newx > 0 && newy <= n
      pos(7,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(7) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(7) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;
      end
      cost(7) = costsofar + sqrt(2)*field(newy,newx);

    end
    
    % 拓展方向八
    newx = currentpos(2)-1; newy = currentpos(1)-1;
    if newx > 0 && newy > 0
      pos(8,:) = [newy newx];
      switch lower(heuristicmethod)
        case 'euclidean'
          heuristic(8) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) / bn;

        case 'taxicab'
          heuristic(8) = (abs(goalpos(2)-newx) + abs(goalpos(1)-newy)) ;

      end
      cost(8) = costsofar + sqrt(2)*field(newy,newx);

    end
     posinds = sub2ind([n n],pos(:,1),pos(:,2)); % 将拓展出来的子节点的 坐标值 转换为 索引值


% --- Executes on button press in ALLpathsfind.
function ALLpathsfind_Callback(hObject, eventdata, handles)
% hObject    handle to ALLpathsfind (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tic
global   maze mapdata mapspeed maprisk 
G = maze;
D = mapdata;
F = mapspeed;
R = maprisk;
A = str2double(get(handles.row,'string'));

shangebianchang=str2double(get(handles.lengthofperblock,'string'));
normalspeed=str2double(get(handles.normalspeed,'string'));
topnb=shangebianchang/normalspeed;    %每个平地方格消耗时间
secost=str2double(get(handles.normalcost,'string'));
serisk=str2double(get(handles.normalrisk,'string'));
sespeed=topnb;

% DFS_Maze(maze,mapdata,mapfar,maprisk,secost,serisk,selength,Ablocknumber);
DFS_Maze(G,D,F,R,secost,serisk,sespeed,A);
toc
time=toc;

set(handles.time,'string',time);

str1 = [ ];
str2 = [ ];
str3 = [ ];
set(handles.textshow1,'string',str1);
set(handles.textshow2,'string',str2);
set(handles.textshow3,'string',str3);


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  mapmatrix mapdata mapfar maprisk  maze mapspeed mapG
save('mapmatrix.mat','mapmatrix');    %其中name是要存储的名字，data是要存储的矩阵，前者是输出，后者输入。
save('mapdata.mat','mapdata');
save('mapfar.mat','mapfar');
save('maprisk.mat','maprisk');
save('maze.mat','maze');
save('mapspeed.mat','mapspeed');
save('mapG.mat','mapG');


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  mapmatrix mapdata mapfar maprisk  maze phandles mapspeed startflag   edittype SandEpoint  mapz mapG
load('mapmatrix.mat','mapmatrix');   
load('mapdata.mat','mapdata');
load('mapfar.mat','mapfar');
load('maprisk.mat','maprisk');
load('maze.mat','maze');
load('mapspeed.mat','mapspeed');
load('mapG.mat','mapG');


     row = str2double(get(handles.row,'string'));
    column = str2double(get(handles.column,'string'));
    map=[1 1 1;  %白
        0 0 1;   %蓝
        1 0 1;   %品红
        0 0 0;   %黑
        1 0 0;   %红
        1 1 0;   %黄
        0.46667 0.53333 0.6      %LightSlateGray 
        0.41176 0.41176 0.41176  %DimGrey
        0 0.74902 1              %DeepSkyBlue
        0 1 0];  %绿
    colormap(map);
    phandles = pcolor(1:row+1,1:column+1,[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]);
    set(phandles,'ButtonDownFcn',{@mapClickCallback,handles});   


if startflag
    shangebianchang=str2double(get(handles.lengthofperblock,'string'));  %每个栅格边长
    hightofnormal=str2double(get(handles.hightofnormal,'string'));       %平台高度
    upladderhight=str2double(get(handles.hightofupladder,'string'));     %竖梯高度
    hightofsmallwall=str2double(get(handles.hightofsmallwall,'string')); %矮墙高度
    hightofwall=str2double(get(handles.hightofwall,'string'));           %墙高度
    breadthoflittleriver=str2double(get(handles.breadthoflittleriver,'string')); %小溪宽度
    
    normalspeed=str2double(get(handles.normalspeed,'string'));     %平地行进速度
    climbspeed=str2double(get(handles.upladderspeed,'string'));    %爬梯子速度
    swingspeed=str2double(get(handles.ladderspeed,'string'));      %摆荡速度
    anglespeed=str2double(get(handles.anglespeed,'string'));       %斜坡速度
    jumpspeed=str2double(get(handles.jumpspeedperm,'string'));     %跳高速度
    throwoverspeed=str2double(get(handles.throwoverspeed,'string'));   %翻越速度
    longjumpspeed=str2double(get(handles.longjumpspeed,'string'));    %跳远速度
    
    
    topnb=shangebianchang/normalspeed;    %每个平地方格消耗时间
    toplb=shangebianchang/swingspeed;     %摆荡时间
    topab=shangebianchang/anglespeed;     %斜坡时间
    tocu=upladderhight/climbspeed;        %爬梯子时间
    tojump=hightofsmallwall/jumpspeed;    %跳高时间
    tothrowover=hightofwall/throwoverspeed; %翻越时间
    tolongjump=breadthoflittleriver/longjumpspeed; %跳远时间
    

    
    
    axesHandle = get(hObject,'Parent');
    temp = get(axesHandle,'CurrentPoint');%获取最近一次点击的位置
    coordinates = floor(temp(1,1:2));
    switch edittype
        case 1     %平地
            mapmatrix(coordinates(2),coordinates(1)) =1; %白色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.normalcost,'string'))*shangebianchang; 
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.normalrisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = topnb;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = hightofnormal ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 2     %高墙
            mapmatrix(coordinates(2),coordinates(1)) =4; %黑色
            mapdata(coordinates(2),coordinates(1)) = inf;
            mapfar (coordinates(2),coordinates(1)) = inf;
            maprisk(coordinates(2),coordinates(1)) = inf;
            mapspeed(coordinates(2),coordinates(1)) = inf;
            maze(coordinates(2),coordinates(1)) = 1 ;
            mapz(coordinates(2),coordinates(1)) = inf;
            mapG(coordinates(2),coordinates(1)) = 1 ;
        case 3     %河
            mapmatrix(coordinates(2),coordinates(1)) =2; %蓝色
            mapdata(coordinates(2),coordinates(1)) = inf;
            mapfar (coordinates(2),coordinates(1)) = inf;
            maprisk(coordinates(2),coordinates(1)) = inf;
            mapspeed(coordinates(2),coordinates(1)) = inf;
            maze(coordinates(2),coordinates(1)) = 1 ;
            mapz(coordinates(2),coordinates(1)) = inf;
            mapG(coordinates(2),coordinates(1)) = 1 ;
        case 4     %斜面
            mapmatrix(coordinates(2),coordinates(1)) = 3; %品红
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.anglecost,'string'))*shangebianchang;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.anglerisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = topab;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 5     %云梯
            mapmatrix(coordinates(2),coordinates(1)) = 5; %红色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.laddercost,'string'))*shangebianchang;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.ladderrisk,'string'))*shangebianchang;
            mapspeed(coordinates(2),coordinates(1)) = toplb;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 6
            mapmatrix(SandEpoint(1,2),SandEpoint(1,1)) = 1;
            SandEpoint(1,:) = coordinates;
            mapmatrix(coordinates(2),coordinates(1)) = 10;
            maze(coordinates(2),coordinates(1)) = 2 ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 7
            mapmatrix(SandEpoint(2,2),SandEpoint(2,1)) = 1;
            SandEpoint(2,:) = coordinates;
            mapmatrix(coordinates(2),coordinates(1)) = 10;
            maze(coordinates(2),coordinates(1)) = 3 ;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 8    %竖梯
            mapmatrix(coordinates(2),coordinates(1)) = 6; %黄色
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.upladdercost,'string'))*upladderhight;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.upladderrisk,'string'))*upladderhight;
            mapspeed(coordinates(2),coordinates(1)) = tocu;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = upladderhight;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 9    %矮墙
            mapmatrix(coordinates(2),coordinates(1)) = 7; %LightSlateGray 
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costofjumpperm,'string'))*hightofsmallwall;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.jumpriskperm,'string'))*hightofsmallwall;
            mapspeed(coordinates(2),coordinates(1)) = tojump;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 10    %墙
            mapmatrix(coordinates(2),coordinates(1)) = 8; %DimGrey
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costofthrowover,'string'))*hightofwall;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.throwoverrisk,'string'))*hightofwall;
            mapspeed(coordinates(2),coordinates(1)) = tothrowover;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        case 11    %溪流
            mapmatrix(coordinates(2),coordinates(1)) = 9; %DeepSkyBlue
            mapdata(coordinates(2),coordinates(1)) = str2double(get(handles.costoflongjumpperm,'string'))*breadthoflittleriver;
            mapfar (coordinates(2),coordinates(1)) = shangebianchang;
            maprisk(coordinates(2),coordinates(1)) = str2double(get(handles.riskoflongjumpperm,'string'))*breadthoflittleriver;
            mapspeed(coordinates(2),coordinates(1)) = tolongjump;
            maze(coordinates(2),coordinates(1)) = 0 ;
            mapz(coordinates(2),coordinates(1)) = 0;
            mapG(coordinates(2),coordinates(1)) = 0 ;
        otherwise
            return;
    end

    row = str2double(get(handles.row,'string'));
    column = str2double(get(handles.column,'string'));
    map=[1 1 1;  %白
        0 0 1;   %蓝
        1 0 1;   %品红
        0 0 0;   %黑
        1 0 0;   %红
        1 1 0;   %黄
        0.46667 0.53333 0.6      %LightSlateGray 
        0.41176 0.41176 0.41176  %DimGrey
        0 0.74902 1              %DeepSkyBlue
        0 1 0];  %绿
    colormap(map);
    phandles = pcolor(1:row+1,1:column+1,[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]);
    set(phandles,'ButtonDownFcn',{@mapClickCallback,handles}); 
end


 


% --- Executes during object creation, after setting all properties.
function pushbutton12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function textshow2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to textshow2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function normalspeed_Callback(hObject, eventdata, handles)
% hObject    handle to normalspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of normalspeed as text
%        str2double(get(hObject,'String')) returns contents of normalspeed as a double


% --- Executes during object creation, after setting all properties.
function normalspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to normalspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function anglespeed_Callback(hObject, eventdata, handles)
% hObject    handle to anglespeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of anglespeed as text
%        str2double(get(hObject,'String')) returns contents of anglespeed as a double


% --- Executes during object creation, after setting all properties.
function anglespeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to anglespeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ladderspeed_Callback(hObject, eventdata, handles)
% hObject    handle to ladderspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ladderspeed as text
%        str2double(get(hObject,'String')) returns contents of ladderspeed as a double


% --- Executes during object creation, after setting all properties.
function ladderspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ladderspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function upladderspeed_Callback(hObject, eventdata, handles)
% hObject    handle to upladderspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of upladderspeed as text
%        str2double(get(hObject,'String')) returns contents of upladderspeed as a double


% --- Executes during object creation, after setting all properties.
function upladderspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to upladderspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function hightofupladder_Callback(hObject, eventdata, handles)
% hObject    handle to hightofupladder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hightofupladder as text
%        str2double(get(hObject,'String')) returns contents of hightofupladder as a double


% --- Executes during object creation, after setting all properties.
function hightofupladder_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hightofupladder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in smallwall.
function smallwall_Callback(hObject, eventdata, handles)
% hObject    handle to smallwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of smallwall
global edittype
if get(handles.smallwall,'value')
    edittype = 9;
    set(handles.upladder,'value',0);
    set(handles.ladder,'value',0);
    set(handles.angle,'value',0);
    set(handles.normalground,'value',0); 
    set(handles.wall,'value',0);
    set(handles.tallwall,'value',0);
    set(handles.littleriver,'value',0);
    set(handles.river,'value',0);
    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
end


function hightofwall_Callback(hObject, eventdata, handles)
% hObject    handle to hightofwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hightofwall as text
%        str2double(get(hObject,'String')) returns contents of hightofwall as a double


% --- Executes during object creation, after setting all properties.
function hightofwall_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hightofwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function hightofsmallwall_Callback(hObject, eventdata, handles)
% hObject    handle to hightofsmallwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hightofsmallwall as text
%        str2double(get(hObject,'String')) returns contents of hightofsmallwall as a double


% --- Executes during object creation, after setting all properties.
function hightofsmallwall_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hightofsmallwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in tallwall.
function tallwall_Callback(hObject, eventdata, handles)
% hObject    handle to tallwall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tallwall
global edittype 
if get(handles.tallwall,'value')
    edittype = 2;
   set(handles.upladder,'value',0);
   set(handles.ladder,'value',0);
   set(handles.angle,'value',0);
   set(handles.normalground,'value',0);  
   set(handles.smallwall,'value',0);
   set(handles.wall,'value',0);
   set(handles.littleriver,'value',0);
   set(handles.river,'value',0);
   set(handles.startpoint,'value',0);
   set(handles.endpoint,'value',0);
end

% --- Executes on button press in littleriver.
function littleriver_Callback(hObject, eventdata, handles)
% hObject    handle to littleriver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of littleriver
global edittype 
if get(handles.littleriver,'value')
    edittype = 11;
    set(handles.upladder,'value',0);
    set(handles.ladder,'value',0);
    set(handles.angle,'value',0);
    set(handles.normalground,'value',0);  
    set(handles.smallwall,'value',0);
    set(handles.wall,'value',0);
    set(handles.tallwall,'value',0);
    set(handles.river,'value',0);
    set(handles.startpoint,'value',0);
    set(handles.endpoint,'value',0);
end


function jumpriskperm_Callback(hObject, eventdata, handles)
% hObject    handle to jumpriskperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of jumpriskperm as text
%        str2double(get(hObject,'String')) returns contents of jumpriskperm as a double


% --- Executes during object creation, after setting all properties.
function jumpriskperm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to jumpriskperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function costofjumpperm_Callback(hObject, eventdata, handles)
% hObject    handle to costofjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of costofjumpperm as text
%        str2double(get(hObject,'String')) returns contents of costofjumpperm as a double


% --- Executes during object creation, after setting all properties.
function costofjumpperm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to costofjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function throwoverrisk_Callback(hObject, eventdata, handles)
% hObject    handle to throwoverrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of throwoverrisk as text
%        str2double(get(hObject,'String')) returns contents of throwoverrisk as a double


% --- Executes during object creation, after setting all properties.
function throwoverrisk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to throwoverrisk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function costofthrowover_Callback(hObject, eventdata, handles)
% hObject    handle to costofthrowover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of costofthrowover as text
%        str2double(get(hObject,'String')) returns contents of costofthrowover as a double


% --- Executes during object creation, after setting all properties.
function costofthrowover_CreateFcn(hObject, eventdata, handles)
% hObject    handle to costofthrowover (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function throwoverspeed_Callback(hObject, eventdata, handles)
% hObject    handle to throwoverspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of throwoverspeed as text
%        str2double(get(hObject,'String')) returns contents of throwoverspeed as a double


% --- Executes during object creation, after setting all properties.
function throwoverspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to throwoverspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function jumpspeedperm_Callback(hObject, eventdata, handles)
% hObject    handle to jumpspeedperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of jumpspeedperm as text
%        str2double(get(hObject,'String')) returns contents of jumpspeedperm as a double


% --- Executes during object creation, after setting all properties.
function jumpspeedperm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to jumpspeedperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function longjumpspeed_Callback(hObject, eventdata, handles)
% hObject    handle to longjumpspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of longjumpspeed as text
%        str2double(get(hObject,'String')) returns contents of longjumpspeed as a double


% --- Executes during object creation, after setting all properties.
function longjumpspeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to longjumpspeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function riskoflongjumpperm_Callback(hObject, eventdata, handles)
% hObject    handle to riskoflongjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of riskoflongjumpperm as text
%        str2double(get(hObject,'String')) returns contents of riskoflongjumpperm as a double


% --- Executes during object creation, after setting all properties.
function riskoflongjumpperm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to riskoflongjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function costoflongjumpperm_Callback(~, eventdata, handles)
% hObject    handle to costoflongjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of costoflongjumpperm as text
%        str2double(get(hObject,'String')) returns contents of costoflongjumpperm as a double


% --- Executes during object creation, after setting all properties.
function costoflongjumpperm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to costoflongjumpperm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function breadthoflittleriver_Callback(hObject, eventdata, handles)
% hObject    handle to breadthoflittleriver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of breadthoflittleriver as text
%        str2double(get(hObject,'String')) returns contents of breadthoflittleriver as a double


% --- Executes during object creation, after setting all properties.
function breadthoflittleriver_CreateFcn(hObject, eventdata, handles)
% hObject    handle to breadthoflittleriver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function hightofnormal_Callback(hObject, eventdata, handles)
% hObject    handle to hightofnormal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hightofnormal as text
%        str2double(get(hObject,'String')) returns contents of hightofnormal as a double


% --- Executes during object creation, after setting all properties.
function hightofnormal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hightofnormal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  mapmatrix mapdata mapfar maprisk  maze mapspeed mapz
nr=str2double(get(handles.normalrisk,'string'));
nc=str2double(get(handles.normalcost,'string'));
shangebianchang=str2double(get(handles.lengthofperblock,'string'));
normalspeed=str2double(get(handles.normalspeed,'string'));
topnb=shangebianchang/normalspeed;    %每个平地方格消耗时间


posofstartgoal=find(mapmatrix==10);
posofstart=posofstartgoal(1);
posofgoal=posofstartgoal(2);
mapmatrix(posofstart)=1;
mapmatrix(posofgoal)=1;
mapdata(posofstart)=nc;
mapdata(posofgoal)=nc;
maprisk(posofstart)=nr;
maprisk(posofgoal)=nr;
mapfar(posofstart)=shangebianchang;
mapfar(posofgoal)=shangebianchang;
maze(posofstart)=0;
maze(posofgoal)=0;
mapspeed(posofstart)=topnb;
mapspeed(posofgoal)=topnb;
mapz(posofstart)=0;
mapz(posofgoal)=0;


    row = str2double(get(handles.row,'string'));
    column = str2double(get(handles.column,'string'));
    map=[1 1 1;  %白
        0 0 1;   %蓝
        1 0 1;   %品红
        0 0 0;   %黑
        1 0 0;   %红
        1 1 0;   %黄
        0.46667 0.53333 0.6      %LightSlateGray 
        0.41176 0.41176 0.41176  %DimGrey
        0 0.74902 1              %DeepSkyBlue
        0 1 0];  %绿
    colormap(map);
    phandles = pcolor(1:row+1,1:column+1,[mapmatrix mapmatrix(:,end); mapmatrix(end,:) mapmatrix(end,end)]);
    set(phandles,'ButtonDownFcn',{@mapClickCallback,handles}); 


% --- Executes on button press in GApathfinding.
function GApathfinding_Callback(~, eventdata, handles)
% hObject    handle to GApathfinding (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tic
global  mapG  mapmatrix

%GA与gui索引起点不同排列不同，所以将索引对齐
n = str2double(get(handles.row,'string'));
indexeslimit=n*n-1;
indexeslist=0:indexeslimit;
mapindex1=reshape(indexeslist,[n,n]);
mapindex2=mapindex1';
sgpoint=find(mapmatrix==10);
New_startposind=sgpoint(1); 
New_goalposind=sgpoint(2);        
start=mapindex2(New_startposind);
goal=mapindex2(New_goalposind);               

%调用GA
GApathfinding(mapG,start,goal,n)  

toc
time=toc;
set(handles.time,'string',time);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mapG mapspeed maprisk mapdata 

ant_colony(mapG,mapspeed,maprisk,mapdata)
