function varargout = SensorAnalyzer(varargin)
% SENSORANALYZER MATLAB code for SensorAnalyzer.fig
%      SENSORANALYZER, by itself, creates a new SENSORANALYZER or raises the existing
%      singleton*.
%
%      H = SENSORANALYZER returns the handle to a new SENSORANALYZER or the handle to
%      the existing singleton*.
%
%      SENSORANALYZER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SENSORANALYZER.M with the given input arguments.
%
%      SENSORANALYZER('Property','Value',...) creates a new SENSORANALYZER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SensorAnalyzer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SensorAnalyzer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SensorAnalyzer

% Last Modified by GUIDE v2.5 08-Nov-2017 14:53:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SensorAnalyzer_OpeningFcn, ...
                   'gui_OutputFcn',  @SensorAnalyzer_OutputFcn, ...
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


% --- Executes just before SensorAnalyzer is made visible.
function SensorAnalyzer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SensorAnalyzer (see VARARGIN)

% Choose default command line output for SensorAnalyzer
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SensorAnalyzer wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SensorAnalyzer_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function logFilePath_Callback(hObject, eventdata, handles)
% hObject    handle to logFilePath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of logFilePath as text
%        str2double(get(hObject,'String')) returns contents of logFilePath as a double


% --- Executes during object creation, after setting all properties.
function logFilePath_CreateFcn(hObject, eventdata, handles)
% hObject    handle to logFilePath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in selectPath.
function selectPath_Callback(hObject, eventdata, handles)
% hObject    handle to selectPath (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( ...
{  '*.mat','MAT文件 (*.mat)'; ...
   '*.zf;','机载日志 (*.zf)'; ...
   '*.*',  '所有文件(*.*)'}, ...
   '选择日志文件');
if isequal(filename,0)
    return;
end
fileFullPath = strcat(pathname,filename);
handles.logFilePath.String = fileFullPath;
handles.FileFullPath = fileFullPath;
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in analyzeIMU.
function analyzeIMU_Callback(hObject, eventdata, handles)
% hObject    handle to analyzeIMU (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FileFullPath = handles.logFilePath.String;
if isempty(FileFullPath)
    warndlg('请选择文件','提示','modal');
    return;
end
if ~exist(FileFullPath)
    warndlg('文件不存在','提示','modal');
    return;
end

mat = load(FileFullPath);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fid=fopen([FileFullPath(1:end-6),'dat'],'w');
fprintf(fid,'IMU GyrX  GyrY  GyrZ  AccX  AccY  AccZ \n');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
count = 0;
error1 = 0;
error2 = 0;
error3 = 0;
error4 = 0;
error5 = 0;
error6 = 0;
len_temp=min(length(mat.IMU(:, 2)),length(mat.IMU2(:, 2)));
for i = 1 :len_temp 
    error1 = error1 + mat.IMU(i, 3) - mat.IMU2(i, 3);
    error2 = error2 + mat.IMU(i, 4) - mat.IMU2(i, 4);
    error3 = error3 + mat.IMU(i, 5) - mat.IMU2(i, 5);
    error4 = error4 + mat.IMU(i, 6) - mat.IMU2(i, 6);
    error5 = error5 + mat.IMU(i, 7) - mat.IMU2(i, 7);
    error6 = error6 + mat.IMU(i, 8) - mat.IMU2(i, 8);
    count = count + 1;
end;

error1 = error1 / count;
error2 = error2 / count;
error3 = error3 / count;
error4 = error4 / count;
error5 = error5 / count;
error6 = error6 / count;

if (abs(error1) > 0.2) || (abs(error2) > 0.2) || (abs(error3) > 0.2) || (abs(error4) > 0.3) || (abs(error5) > 0.3) || (abs(error6) > 0.5)
    warndlg('IMU异常！！！！！！！！！！！！','警告','error');
%     return;
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'IMU1_IMU2 ');
data_ck=[error1 error2 error3 error4 error5 error6];
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

count = 0;
error1 = 0;
error2 = 0;
error3 = 0;
error4 = 0;
error5 = 0;
error6 = 0;
len_temp=min(length(mat.IMU(:, 2)),length(mat.IMU3(:, 2)));
for i = 1 : len_temp
    error1 = error1 + mat.IMU(i, 3) - mat.IMU3(i, 3);
    error2 = error2 + mat.IMU(i, 4) - mat.IMU3(i, 4);
    error3 = error3 + mat.IMU(i, 5) - mat.IMU3(i, 5);
    error4 = error4 + mat.IMU(i, 6) - mat.IMU3(i, 6);
    error5 = error5 + mat.IMU(i, 7) - mat.IMU3(i, 7);
    error6 = error6 + mat.IMU(i, 8) - mat.IMU3(i, 8);
    count = count + 1;
end;

error1 = error1 / count;
error2 = error2 / count;
error3 = error3 / count;
error4 = error4 / count;
error5 = error5 / count;
error6 = error6 / count;

if (abs(error1) > 0.2) || (abs(error2) > 0.2) || (abs(error3) > 0.2) || (abs(error4) > 0.3) || (abs(error5) > 0.3) || (abs(error6) > 0.5)
    warndlg('IMU异常！！！！！！！！！！！！','警告','error');
%     return;
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'IMU1_IMU3 ');
data_ck=[error1 error2 error3 error4 error5 error6];
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

count = 0;
error1 = 0;
error2 = 0;
error3 = 0;
error4 = 0;
error5 = 0;
error6 = 0;
len_temp=min(length(mat.IMU2(:, 2)),length(mat.IMU3(:, 2)));
for i = 1 : len_temp
    error1 = error1 + mat.IMU3(i, 3) - mat.IMU2(i, 3);
    error2 = error2 + mat.IMU3(i, 4) - mat.IMU2(i, 4);
    error3 = error3 + mat.IMU3(i, 5) - mat.IMU2(i, 5);
    error4 = error4 + mat.IMU3(i, 6) - mat.IMU2(i, 6);
    error5 = error5 + mat.IMU3(i, 7) - mat.IMU2(i, 7);
    error6 = error6 + mat.IMU3(i, 8) - mat.IMU2(i, 8);
    count = count + 1;
end;

error1 = error1 / count;
error2 = error2 / count;
error3 = error3 / count;
error4 = error4 / count;
error5 = error5 / count;
error6 = error6 / count;

if (abs(error1) > 0.2) || (abs(error2) > 0.2) || (abs(error3) > 0.2) || (abs(error4) > 0.3) || (abs(error5) > 0.3) || (abs(error6) > 0.5)
    warndlg('IMU异常！！！！！！！！！！！！','警告','error');
%     return;
end;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf(fid,'IMU2_IMU3 ');
data_ck=[error1 error2 error3 error4 error5 error6];
    [count,num]=size(data_ck);
    for i=1:count
        for j=1:num
            fprintf(fid,'%f ',data_ck(i,j));
        end
        fprintf(fid,'\n');
    end
    fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

warndlg('IMU正常','结论','custom');
guidata(hObject, handles);


% --- Executes on button press in gpsNsats.
function gpsNsats_Callback(hObject, eventdata, handles)
% hObject    handle to gpsNsats (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FileFullPath = handles.logFilePath.String;
if isempty(FileFullPath)
    warndlg('请选择文件','提示','modal');
    return;
end
if ~exist(FileFullPath)
    warndlg('文件不存在','提示','modal');
    return;
end

mat = load(FileFullPath);

zerocount = 0;
count = 0;
totalsat = 0;
minsat = 100;
for i = 1 : length(mat.GPS(:, 2))
    totalsat = totalsat + mat.GPS(i, 5);
    count = count + 1;
    if mat.GPS(i, 5) == 0
        zerocount = zerocount + 1;
    end;
    if (minsat > mat.GPS(i, 5))
        minsat = mat.GPS(i, 5);
    end;
end;
totalsat = totalsat / count;
figure(1);
plot(mat.GPS(:, 14), mat.GPS(:, 5));
if zerocount ~= 0
    warndlg('星数跳过0！！！！！！！！！！！！！！','警告','error');
    return;
end;
strr = '平均星数：';
strr = strcat(strr, num2str(fix(totalsat)));
strr = strcat(strr, ' 最少星数：');
strr = strcat(strr, num2str(fix(minsat)));
strr = strcat(strr, '需要根据曲线再确认一遍');
warndlg(strr,'提示','custom');


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FileFullPath = handles.logFilePath.String;
if isempty(FileFullPath)
    warndlg('请选择文件','提示','modal');
    return;
end
if ~exist(FileFullPath)
    warndlg('文件不存在','提示','modal');
    return;
end

mat = load(FileFullPath);

startNum = 0;
endNum = 0;

for i=1:length(mat.MSG(:, 2))
    if (mat.MSG(i, 2) == 1006)  startNum = mat.MSG(i, 1);   end;
    if (mat.MSG(i, 2) == 1001)  endNum = mat.MSG(i, 1);     end;
end;

errorPitch = [];
att = [];
datt = [];
for i=1:length(mat.ATT(:, 2))
    if (mat.ATT(i, 1)>startNum && mat.ATT(i, 1)<endNum)
        errorPitch = [errorPitch, (mat.ATT(i, 5)-mat.ATT(i, 6))/100];
        att = [att, mat.ATT(i, 6)/100];
        datt = [datt, mat.ATT(i, 5)/100];
    end;
end;

strr = '起飞后到返航前俯仰误差：';
strr = strcat(strr, num2str(mean(errorPitch)));
strr = strcat(strr, '±');
strr = strcat(strr, num2str(std(errorPitch)));

warndlg(strr,'提示','custom');
figure(1);
plot(1:length(att), att, 1:length(att), datt);
