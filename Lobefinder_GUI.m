function varargout = Lobefinder_GUI(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Lobefinder_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Lobefinder_GUI_OutputFcn, ...
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
end
% End initialization code - DO NOT EDIT

function p=parameter_setting(p)
% ---[parameter setting]
p.gap_k=3;                    % gap_k: lobe adjacent threshold. If gap_k=1, i-1,i,i+1 are in the same lobe.     
p.h_width_ratio=0.3;          % h_width_ratio:  concave height/width threshold. If h<gap_height, points are belong to the same loop. 
%p.h_radius_ratio=0.05;        % h_radius_ratio: concave height/radius threshold. 
p.t1='t1';        % target cell at t1 stage(nx2)='h38'
p.t2='t2';        % target cell at t1 stage(nx2)='h55'
p.section_number=100;       % intepration spline2 parameter(section number)
p.d=0;                      % intepration spline2 parameter(intepration distance)
p.radius=20;                %p.radius: resize setting radius
p.pixel_di_um=2.55;         %2.55 pixels/痠

% peakfinder parameters
p.sel=600; %sel - The amount above surrounding data for a peak to be identified (default = (max(x0)-min(x0))/4). Larger values mean the algorithm is more selective in finding peaks.

% diff2 parameters
p.diff2_n=15;  % moving average(15) of in.
p.diff2_thread=1; %thread of second diff, >thread means inverse point 

% cell morphology expansion x y
p.expan_x=1;
p.expan_y=1;

% lobe_position Accuracy
p.correct_threa=pi/20;

p.excel_name='Lobe_result';

p=macorwindow(p);

end

% --- Executes just before Lobefinder_GUI is made visible.
function Lobefinder_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Lobefinder_GUI (see VARARGIN)

% Choose default command line output for Lobefinder_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
end

% UIWAIT makes Lobefinder_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Lobefinder_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)

% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
p=handles.p;
i=get(handles.listbox1,'Value');
Lob_xy=handles.Lob_xy;
Lob_Rlist=handles.Lob_Rlist;
A=[];
eval(['A=Lob_Rlist{' num2str(i) '};']);
eval([ 'set(handles.uitable1,''data'',[ Lob_Rlist{' num2str(i) '}.x_cell(1,Lob_Rlist{' num2str(i) '}.klobe_e(:,2))'', Lob_Rlist{' num2str(i) '}.y_cell(1,Lob_Rlist{' num2str(i) '}.klobe_e(:,2))''])' ]);
eval([  'set(handles.edit1,''String'',num2str(A.klobe_num));'  ]);
eval([  'set(handles.edit2,''String'',num2str(A.compactness));'  ]);
eval([  'set(handles.edit3,''String'',num2str(A.roundness));'  ]);
eval([  'set(handles.edit4,''String'',num2str(A.convexity));'  ]);
eval([  'set(handles.edit5,''String'',num2str(A.solidity));'  ]);

%eval(['xy_full=[A.x_cell;A.y_cell];']);xy_full=[xy_full xy_full(:,1)];
eval(['xy_full=A.xy_o(1:2,:)/p.pixel_di_um;']);xy_full=[xy_full xy_full(:,1)];
eval(['plot(handles.axes1,'  'xy_full(1,:), xy_full(2,:),A.x_cell(A.klobe2(:,2)),A.y_cell(A.klobe2(:,2)),''--k'',A.xy_o(1,A.klobe(:,1))/p.pixel_di_um,A.xy_o(2,A.klobe(:,1))/p.pixel_di_um,''ro'',A.x_cell(1,A.klobe_e(:,2)),A.y_cell(1,A.klobe_e(:,2)),''k*'')'] );
xy_max=max(xy_full(1:2,:)');xy_min=min(xy_full(1:2,:)');
if xy_max(1)-xy_min(1)>=(xy_max(2)-xy_min(2))*14.4/10
    dd=xy_max(1)*1.05-xy_min(1)*1.05;%ddd=(dd-xy_max(2)+xy_min(2))/2;
    ddd=(dd/14.4*10-(xy_max(2)-xy_min(2)))/2;
    axis(handles.axes1,[xy_min(1)*1.05 xy_max(1)*1.05 xy_min(2)-ddd xy_max(2)+ddd]);
    set(handles.axes1,'XTick',ceil(xy_min(1)*1.05/10)*10:10:floor(xy_max(1)*1.05/10)*10, 'YTick',ceil((xy_min(2)-ddd)/10)*10:10:floor((xy_max(2)+ddd)/10)*10);
else
    dd=xy_max(2)*1.05-xy_min(2)*1.05;%ddd=(dd-xy_max(1)+xy_min(1))/2;
    ddd=(dd*14.4/10-(xy_max(1)-xy_min(1)))/2;
    axis(handles.axes1,[xy_min(1)-ddd xy_max(1)+ddd xy_min(2)*1.05 xy_max(2)*1.05]);
    set(handles.axes1,'XTick',ceil((xy_min(1)-ddd)/10)*10:10:floor((xy_max(1)+ddd)/10)*10, 'YTick',ceil(xy_min(2)*1.05/10)*10:10:floor(xy_max(2)*1.05/10)*10);
end
eval(['title(handles.axes1,'' Image: ' Lob_xy{i,2} ''')']);
legend(handles.axes1,'cell wall', 'refined convex hull','control points','predicted lobe points','Location','Best');

eval(['plot(handles.axes2,1:size(A.rev_distancesReal,2),A.rev_distancesReal,A.klobe_e(:,3),A.rev_distancesReal(1,A.klobe_e(:,3)),''r*'');']);
ylabel(handles.axes2,'DTRH(um)');xlabel(handles.axes2,'Hull perimeter(um)');
xnum_max=size(A.rev_distancesReal,2);
x_label_data=0:round(xnum_max/500)*50:xnum_max;
set(handles.axes2,'XTick',x_label_data);x_label_data_txt='{''';
for xx=1:size(x_label_data,2)-1
    x_label_data_txt=[x_label_data_txt num2str(x_label_data(xx)/5) ''','''];
end
x_label_data_txt=[x_label_data_txt num2str(x_label_data(size(x_label_data,2))/5) '''}'];
eval(['set(handles.axes2,''XTickLabel'',' x_label_data_txt ');']);
set(handles.axes2,'Xlim',[1,size(A.rev_distancesReal,2)]);
legend(handles.axes2,'DTRH', 'predicted lobe points','Location','Best');

set(gcf,'WindowButtonMotionFcn',{@windowbuttondownfunc,xy_full,A,handles});


    function windowbuttondownfunc(hobj,event,xy_full,A,handles)     
        if strcmp(get(gcf,'selectiontype'),'normal')
            i=get(handles.listbox1,'Value');
            Lob_xy=handles.Lob_xy;
            cp=get(gcf,'CurrentPoint');
            pos=get(handles.axes1,'position');
            x_lim=get(handles.axes1,'xlim');y_lim=get(handles.axes1,'ylim');
            x1=x_lim(1)+(cp(1)-pos(1))*(x_lim(2)-x_lim(1))/pos(3);
            y1=y_lim(1)+(cp(2)-pos(2))*(y_lim(2)-y_lim(1))/pos(4);
            p=handles.p;
            
            n1=size(A.x_cell,2);
            dist3=zeros(1,n1);
            for i1=1:n1
                dist3(1,i1)=sqrt((A.x_cell(1,i1)-x1)^2+(A.y_cell(1,i1)-y1)^2);
            end
            [a1,b1]=min(dist3);b2=round(b1*size(A.rev_distancesReal,2)/n1);
            
            %eval(['plot(handles.axes1,xy_full(1,:), xy_full(2,:),A.x_hull,A.y_hull,''--k'',A.xy_o(1,A.klobe(:,1))/p.pixel_di_um,A.xy_o(2,A.klobe(:,1))/p.pixel_di_um,''ro'',A.x_cell(1,A.klobe_e(:,2)),A.y_cell(1,A.klobe_e(:,2)),''k*'');']);
            eval(['plot(handles.axes1,'  'xy_full(1,:), xy_full(2,:),A.x_cell(A.klobe2(:,2)),A.y_cell(A.klobe2(:,2)),''--k'',A.xy_o(1,A.klobe(:,1))/p.pixel_di_um,A.xy_o(2,A.klobe(:,1))/p.pixel_di_um,''ro'',A.x_cell(1,A.klobe_e(:,2)),A.y_cell(1,A.klobe_e(:,2)),''k*'')'] );
            if a1<2
            hold(handles.axes1,'on')
            eval(['plot(handles.axes1,A.x_cell(1,b1),A.y_cell(1,b1),''g:square'',''MarkerSize'',15,''LineWidth'',2.5);'] );
            hold(handles.axes1,'off')
            if x1>=0
                posti=[num2str(roundn(x1,-2)) '  ' num2str(roundn(y1,-2))];
                set(handles.text17,'visible','on','String',posti,'Position',[cp(1)+5 cp(2)-0.5 18 1.5],'HorizontalAlignment','left');
            else
                posti=[num2str(roundn(x1,-2)) '  ' num2str(roundn(y1,-2))];
                set(handles.text17,'visible','on','String',posti,'Position',[cp(1)-23 cp(2)-0.5 18 1.5],'HorizontalAlignment','right');
            end
            else
                set(handles.text17,'visible','off');           
            end
                xy_max=max(xy_full(1:2,:)');xy_min=min(xy_full(1:2,:)');
                if xy_max(1)-xy_min(1)>=(xy_max(2)-xy_min(2))*14.4/10
                    dd=xy_max(1)*1.05-xy_min(1)*1.05;%ddd=(dd-xy_max(2)+xy_min(2))/2;
                    ddd=(dd/14.4*10-(xy_max(2)-xy_min(2)))/2;
                    axis(handles.axes1,[xy_min(1)*1.05 xy_max(1)*1.05 xy_min(2)-ddd xy_max(2)+ddd]);
                    set(handles.axes1,'XTick',ceil(xy_min(1)*1.05/10)*10:10:floor(xy_max(1)*1.05/10)*10, 'YTick',ceil((xy_min(2)-ddd)/10)*10:10:floor((xy_max(2)+ddd)/10)*10);
                else
                    dd=xy_max(2)*1.05-xy_min(2)*1.05;%ddd=(dd-xy_max(1)+xy_min(1))/2;
                    ddd=(dd*14.4/10-(xy_max(1)-xy_min(1)))/2;
                    axis(handles.axes1,[xy_min(1)-ddd xy_max(1)+ddd xy_min(2)*1.05 xy_max(2)*1.05]);
                    set(handles.axes1,'XTick',ceil((xy_min(1)-ddd)/10)*10:10:floor((xy_max(1)+ddd)/10)*10, 'YTick',ceil(xy_min(2)*1.05/10)*10:10:floor(xy_max(2)*1.05/10)*10);
                end
                eval(['title(handles.axes1,'' Image: ' Lob_xy{i,2} ''')']);
                legend(handles.axes1,'cell wall', 'refined convex hull','control points','predicted lobe points','Location','Best');         
                
            eval(['plot(handles.axes2,1:size(A.rev_distancesReal,2),A.rev_distancesReal,A.klobe_e(:,3),A.rev_distancesReal(1,A.klobe_e(:,3)),''r*'');']);
            if a1<2
            hold(handles.axes2,'on')
            eval(['plot(handles.axes2,b2,A.rev_distancesReal(1,b2),''g:square'',''MarkerSize'',15,''LineWidth'',2.5);']);
            hold(handles.axes2,'off')
            end
            ylabel(handles.axes2,'DTRH(um)');xlabel(handles.axes2,'Hull perimeter(um)');
            exnum_max=size(A.rev_distancesReal,2);
            x_label_data=0:round(xnum_max/500)*50:xnum_max;
            set(handles.axes2,'XTick',x_label_data);x_label_data_txt='{''';
            for xx=1:size(x_label_data,2)-1
                x_label_data_txt=[x_label_data_txt num2str(x_label_data(xx)/5) ''','''];
            end
            x_label_data_txt=[x_label_data_txt num2str(x_label_data(size(x_label_data,2))/5) '''}'];
            eval(['set(handles.axes2,''XTickLabel'',' x_label_data_txt ');']);
            set(handles.axes2,'Xlim',[1,size(A.rev_distancesReal,2)]);
            legend(handles.axes2,'DTRH', 'predicted lobe points','Location','Best');
        end
    end
end

% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
G=cd;
folder_name = uigetdir(G);
handles.folder_name=folder_name;
set(handles.pushbutton2,'enable','on');
guidata(hObject,handles);
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.listbox1,'Value',1);
set(handles.listbox1,'String','Running....');
set(handles.edit1,'String','');
set(handles.edit2,'String','');
set(handles.edit3,'String','');
set(handles.edit4,'String','');
set(handles.edit5,'String','');
set(handles.edit6,'String','');
set(handles.edit7,'String','');
set(handles.uitable1,'data',[]);
guidata(hObject, handles);
handles=Lobefinder2_2(handles);
guidata(hObject, handles);
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
end


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
end


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
end

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end
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
end


function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
end

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
end


function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
end

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
end

% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
g2=get(handles.checkbox1,'value');
if g2==1
    set(handles.popupmenu1,'Visible','on');
    set(handles.text14,'Visible','on');
elseif g2==0
    set(handles.popupmenu1,'Visible','off');
    set(handles.text14,'Visible','off');
end

end


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2
end


function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
end

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
end


function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
end

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
clearvars -global
delete(hObject);
end

%  === Main program ===
function handles=Lobefinder2_2(handles,hObject)
% ------ [Model options] -------------------------------------------------

%1. original_images folder
%p.Data_folder='original_images';
p.Data_folder=handles.folder_name;

%2. Show figures
p.ShowFigures=0; %Show figures when running lobefinder=1, not show=0

%3. Output figures
g1=get(handles.checkbox1,'value');
p.OutputFigures=g1; %save figures as TIFF files=1, not=0

%4. Save as excel file
%g3=get(handles.checkbox3,'value');
%p.OutputExcel=g3; %save output as excel files=1, not=0

g2=get(handles.checkbox2,'value');
g4=get(handles.checkbox4,'value');
p.Outputmat=g2; %save output as excel files=1, not=0
p.Outputtxt=g4;

% -------- [How to use lobefinder 2.1?]-----------------------------------
% How to use Lobefinder 2.1? 
% Step1: Put original image roi files in folder /original_images
% Step2: Change model options(line6~Line12)
% Step3: Run. All lobefinder results will be in 'lobefinder_output.mat'.
% step4: If you would like to change model parameter values: sub-function parameter_setting()

%Edited by George on 5/11/2015

handles=main(p,handles);
end
function handles=main(p,handles,hObject)
disp('run Lobefinder ...');
Lob_xy=ROItoMAT(p.Data_folder);
if iscell(Lob_xy)==0
    eval([  'set(handles.listbox1,''String'',''no roi files'');'  ]);
else
handles.Lob_xy=Lob_xy;
[p]= parameter_setting(p); %% set the parameter and output p struct. DO NOT MODIFY THIS SECTION
p.pixel_di_um=str2num(get(handles.edit10,'String'));
nreplica=size(Lob_xy,1);
eval([  'set(handles.edit6,''String'',num2str(nreplica));'  ]);
axes(handles.axes1);
axes(handles.axes2);
[excel_name,p]=excel_name_prod(p);

for i=1:nreplica
tic;
cellname='Lob';
p.cellname=cellname;
p.replica=i;ii=i;
i=num2str(i);
eval([cellname '_' 'result' i '=ProcessCell('  'Lob_xy{' i ',1}, p,cellname,p.replica)'])

A=[cellname '_' 'result' i];
eval([A '.x_cell=' A '.x_cell/p.pixel_di_um/' A '.ratio_r;']);
eval([A '.y_cell=' A '.y_cell/p.pixel_di_um/' A '.ratio_r;']);
eval([A '.x_hull=' A '.x_hull/p.pixel_di_um/' A '.ratio_r;']);
eval([A '.y_hull=' A '.y_hull/p.pixel_di_um/' A '.ratio_r;']);
eval(['xy_full=' A '.xy_o(1:2,:)/p.pixel_di_um;']);xy_full=[xy_full xy_full(:,1)];

eval(['plot(handles.axes1,'  'xy_full(1,:), xy_full(2,:),' A '.x_cell(' A '.klobe2(:,2)),' A '.y_cell(' A '.klobe2(:,2)),''--k'',' A '.xy_o(1,' A '.klobe(:,1))/p.pixel_di_um,' A '.xy_o(2,' A '.klobe(:,1))/p.pixel_di_um,''ro'',' A '.x_cell(1,' A '.klobe_e(:,2)),' A '.y_cell(1,' A '.klobe_e(:,2)),''k*'')'] );
xy_max=max(xy_full(1:2,:)');xy_min=min(xy_full(1:2,:)');
if xy_max(1)-xy_min(1)>=(xy_max(2)-xy_min(2))*14.4/10
    dd=xy_max(1)*1.05-xy_min(1)*1.05;
    ddd=(dd/14.4*10-(xy_max(2)-xy_min(2)))/2;
    axis(handles.axes1,[xy_min(1)*1.05 xy_max(1)*1.05 xy_min(2)-ddd xy_max(2)+ddd]);
    set(handles.axes1,'XTick',ceil(xy_min(1)*1.05/10)*10:10:floor(xy_max(1)*1.05/10)*10, 'YTick',ceil((xy_min(2)-ddd)/10)*10:10:floor((xy_max(2)+ddd)/10)*10);
else
    dd=xy_max(2)*1.05-xy_min(2)*1.05;
    ddd=(dd*14.4/10-(xy_max(1)-xy_min(1)))/2;
    axis(handles.axes1,[xy_min(1)-ddd xy_max(1)+ddd xy_min(2)*1.05 xy_max(2)*1.05]);
    set(handles.axes1,'XTick',ceil((xy_min(1)-ddd)/10)*10:10:floor((xy_max(1)+ddd)/10)*10, 'YTick',ceil(xy_min(2)*1.05/10)*10:10:floor(xy_max(2)*1.05/10)*10);
end
eval(['title(handles.axes1,'' Image: ' Lob_xy{ii,2} ''')']);
legend(handles.axes1,'cell wall', 'refined convex hull','control points','predicted lobe points','Location','Best');

eval([ 'set(handles.uitable1,''data'',[' A '.x_cell(1,' A '.klobe_e(:,2))'',' A '.y_cell(1,' A '.klobe_e(:,2))''] );' ]);
eval([  'set(handles.edit1,''String'',num2str(' A '.klobe_num));'  ]);
eval([  'set(handles.edit2,''String'',num2str(' A '.compactness));'  ]);
eval([  'set(handles.edit3,''String'',num2str(' A '.roundness));'  ]);
eval([  'set(handles.edit4,''String'',num2str(' A '.convexity));'  ]);
eval([  'set(handles.edit5,''String'',num2str(' A '.solidity));'  ]);
eval([  'set(handles.edit7,''String'',num2str(ii));'  ]);

eval(['plot(handles.axes2,1:size(' A '.rev_distancesReal,2),' A '.rev_distancesReal,' A '.klobe_e(:,3),' A '.rev_distancesReal(1,' A '.klobe_e(:,3)),''r*'');']);
xlabel(handles.axes2,'Hull perimeter(um)');ylabel(handles.axes2,'DTRH(um)');eval(['xnum_max=size(' A '.rev_distancesReal,2);']);
x_label_data=0:round(xnum_max/500)*50:xnum_max;

set(handles.axes2,'XTick',x_label_data);x_label_data_txt='{''';
for xx=1:size(x_label_data,2)-1
    x_label_data_txt=[x_label_data_txt num2str(x_label_data(xx)/5) ''','''];
end
x_label_data_txt=[x_label_data_txt num2str(x_label_data(size(x_label_data,2))/5) '''}'];
eval(['set(handles.axes2,''XTickLabel'',' x_label_data_txt ');']);
eval(['set(handles.axes2,''Xlim'',[1,size(' A '.rev_distancesReal,2)]);']);
legend(handles.axes2,'DTRH', 'predicted lobe points','Location','Best');

handles.p=p;

if p.OutputFigures==1 
   
    f2=figure();set(f2,'Visible','off');
    
    eval(['plot('  'xy_full(1,:), xy_full(2,:),' A '.x_cell(' A '.klobe2(:,2)),' A '.y_cell(' A '.klobe2(:,2)),''--k'',' A '.xy_o(1,' A '.klobe(:,1))/p.pixel_di_um,' A '.xy_o(2,' A '.klobe(:,1))/p.pixel_di_um,''ro'',' A '.x_cell(1,' A '.klobe_e(:,2)),' A '.y_cell(1,' A '.klobe_e(:,2)),''k*'')'] );
    xy_max=max(xy_full(1:2,:)');xy_min=min(xy_full(1:2,:)');
    if xy_max(1)-xy_min(1)>=(xy_max(2)-xy_min(2))*14.4/10
        dd=xy_max(1)*1.05-xy_min(1)*1.05;%ddd=(dd-xy_max(2)+xy_min(2))/2;
        ddd=(dd/14.4*10-(xy_max(2)-xy_min(2)))/2;
        axis([xy_min(1)*1.05 xy_max(1)*1.05 xy_min(2)-ddd xy_max(2)+ddd]);
        set(gca,'XTick',ceil(xy_min(1)*1.05/10)*10:10:floor(xy_max(1)*1.05/10)*10, 'YTick',ceil((xy_min(2)-ddd)/10)*10:10:floor((xy_max(2)+ddd)/10)*10);
    else
        dd=xy_max(2)*1.05-xy_min(2)*1.05;%ddd=(dd-xy_max(1)+xy_min(1))/2;
        ddd=(dd*14.4/10-(xy_max(1)-xy_min(1)))/2;
        axis([xy_min(1)-ddd xy_max(1)+ddd xy_min(2)*1.05 xy_max(2)*1.05]);
        set(gca,'XTick',ceil((xy_min(1)-ddd)/10)*10:10:floor((xy_max(1)+ddd)/10)*10, 'YTick',ceil(xy_min(2)*1.05/10)*10:10:floor(xy_max(2)*1.05/10)*10);
    end
    eval(['title('' Image: ' Lob_xy{ii,2} ''')']);
    legend('cell wall', 'refined convex hull','control points','predicted lobe points','Location','Best');
    
    pp=get(handles.popupmenu1,'Value');
    switch pp
        case 1
            eval([  'print(f2,''-r300'',''-dtiff'',''' A '.tiff'');'  ]);
        case 2
            eval([  'print(f2,''-r300'',''-djpeg'',''' A '.jpeg'');'  ]);
        case 3 
            eval([  'print(f2,''-r100'',''-dbmp'',''' A '.bmp'');'  ]);
        case 4
            eval([  'print(f2,''-r300'',''-dpng'',''' A '.png'');'  ]);
    end    
    close(f2);
    
    f3=figure();set(f3,'Visible','off');
    set(f3, 'Position', [100, 100, 1000, 300]);set(f3, 'PaperPosition', [0, 0, 10, 3]);
    
    eval(['plot(1:size(' A '.rev_distancesReal,2),' A '.rev_distancesReal,' A '.klobe_e(:,3),' A '.rev_distancesReal(1,' A '.klobe_e(:,3)),''r*'');']);
    set(get(gca,'XLabel'),'String','Hull perimeter(um)');set(get(gca,'YLabel'),'String','DTRH(um)');
    eval(['xnum_max=size(' A '.rev_distancesReal,2);']);
    x_label_data=0:round(xnum_max/500)*50:xnum_max;
    set(gca,'XTick',x_label_data);
    set(gca, 'Position', [0.05, 0.13, 0.9, 0.8]);
    x_label_data_txt='{''';
    for xx=1:size(x_label_data,2)-1
        x_label_data_txt=[x_label_data_txt num2str(x_label_data(xx)/5) ''','''];
    end
    x_label_data_txt=[x_label_data_txt num2str(x_label_data(size(x_label_data,2))/5) '''}'];
    eval(['set(gca,''XTickLabel'',' x_label_data_txt ');']);
    eval(['set(gca,''Xlim'',[1,size(' A '.rev_distancesReal,2)]);']);
    legend(gca,'DTRH', 'predicted lobe points','Location','Best');
    
    pp=get(handles.popupmenu1,'Value');
    switch pp
        case 1
            eval([  'print(f3,''-r300'',''-dtiff'',''' A '_dtrh.tiff'');'  ]);
        case 2
            eval([  'print(f3,''-r300'',''-djpeg'',''' A '_dtrh.jpeg'');'  ]);
        case 3 
            eval([  'print(f3,''-r100'',''-dbmp'',''' A '_dtrh.bmp'');'  ]);
        case 4
            eval([  'print(f3,''-r300'',''-dpng'',''' A '_dtrh.png'');'  ]);
    end    
    close(f3);
    
end
if p.Outputtxt==1
eval(['oout=' A]);
    outtxt(A,Lob_xy{ii,1},oout);
end

eval(['listbox_string{ii,1}=' num2str(ii) ';']);
eval([  'handles.Lob_Rlist{' num2str(ii) '}=Lob_result' num2str(ii) ';'  ]);

eval([A '.DTCH=' A '.distances1000;' A '.DTRH=' A '.rev_distances1000;' A '.DTRH_real=' A '.rev_distancesReal;']);
eval(['clear ' A '.distances1000 ' A '.rev_distances1000 ' A '.rev_distancesReal']);
clear G ii
pause(0.1)
toc;
end

set(handles.listbox1,'String',listbox_string);
%% save all data to mat file
end
clear A cellname i nreplica figure1 L1 dd ddd excel_name i listbox_string xy_full xy_max xy_min pp f2 xx xnum_max x_label_data_txt x_label_data
if p.Outputmat==1
    save('Lobe_result.mat')
end
set(handles.listbox1,'Value',1)
listbox1_Callback(handles.listbox1, [], handles);
disp('Finished!!');
end

function COLxy=ROItoMAT(Data_folder)


%% convert data from ROI to xy
%% cite ReadImageJROI :If this code is useful to your (academic) work, please cite the publication in lieu of thanks:
% Muir and Kampa, "FocusStack and StimServer: A new open source MATLAB toolchain for visual stimulation 
% and analysis of two-photon calcium neuronal imaging data". Frontiers in Neuroinformatics (accepted).

%eval(['addpath(''' cd '/' Data_folder ''');']);
%eval(['file=dir(''' cd '/' Data_folder '/*.roi'');']);
eval(['addpath(''' Data_folder ''');']);
eval(['file=dir(''' Data_folder '/*.roi'');']);
if size(file,1)~=0
length(file);
file;


for n=1:length(file)
    Data_folder_files{n,1}=file(n).name;
end

[COL]=ReadImageJROI(Data_folder_files);

for i=1:n
   k=num2str(i);
   eval(['COLxy{' k ',1}=[COL{1, i}.mnCoordinates(:,1),COL{1, i}.mnCoordinates(:,2)];']);
   eval(['COLxy{' k ',2}=Data_folder_files{' k ',1};']);
end 
%% save as a .mat file
%save('COLrawData')
else
    COLxy=0;
end
end
function [sROI] = ReadImageJROI(cstrFilenames)

% ReadImageJROI - FUNCTION Read an ImageJ ROI into a matlab structure
%
% Usage: [sROI] = ReadImageJROI(strFilename)
%        [cvsROIs] = ReadImageJROI(cstrFilenames)
%        [cvsROIs] = ReadImageJROI(strROIArchiveFilename)
%
% This function reads the ImageJ binary ROI file format.
%
% 'strFilename' is the full path to a '.roi' file.  A list of ROI files can be
% passed as a cell array of filenames, in 'cstrFilenames'.  An ImageJ ROI
% archive can be access by providing a '.zip' filename in
% 'strROIArchiveFilename'.  Single ROIs are returned as matlab structures, with
% variable fields depending on the ROI type.  Multiple ROIs are returned as a
% cell array of ROI structures.
%
% The field '.strName' is guaranteed to exist, and contains the ROI name (the
% filename minus '.roi', or the name set for the ROI).
%
% The field '.strType' is guaranteed to exist, and defines the ROI type:
% {'Rectangle', 'Oval', Line', 'Polygon', 'Freehand', 'Traced', 'PolyLine',
% 'FreeLine', 'Angle', 'Point', 'NoROI'}.
%
% The field '.vnRectBounds' is guaranteed to exist, and defines the rectangular
% bounds of the ROI: ['nTop', 'nLeft', 'nBottom', 'nRight'].
%
% The field '.nVersion' is guaranteed to exist, and defines the version number
% of the ROI format.
%
% The field '.vnPosition' is guaranteed to exist. If the information is
% defined within the ROI, this field will be a three-element vector
% [nCPosition nZPosition nTPosition].
%
% ROI types:
%  Rectangle:
%     .strType = 'Rectangle';
%     .nArcSize         - The arc size of the rectangle's rounded corners
%
%      For a composite, 'shape' ROI:
%     .strSubtype = 'Shape';
%     .vfShapeSegments  - A long, complicated vector of complicated shape
%                          segments.  This vector is in the format passed to the
%                          ImageJ ShapeROI constructor.  I won't decode this for
%                          you! :(
%
%  Oval:
%     .strType = 'Oval';
%
%  Line:
%     .strType = 'Line';
%  	.vnLinePoints     - The end points of the line ['nX1', 'nY1', 'nX2', 'nY2']
%
%     With arrow:
%     .strSubtype = 'Arrow';
%     .bDoubleHeaded    - Does the line have two arrowheads?
%     .bOutlined        - Is the arrow outlined?
%     .nArrowStyle      - The ImageJ style of the arrow (unknown interpretation)
%     .nArrowHeadSize   - The size of the arrowhead (unknown units)
%
%  Polygon:
%     .strType = 'Polygon';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the polygon vertices.  Each row is [nX nY].
%
%  Freehand:
%     .strType = 'Freehand';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the polygon vertices.  Each row is [nX nY].
%
%     Ellipse subtype:
%     .strSubtype = 'Ellipse';
%     .vfEllipsePoints  - A vector containing the ellipse control points:
%                          [fX1 fY1 fX2 fY2].
%     .fAspectRatio     - The aspect ratio of the ellipse.
%
%  Traced:
%     .strType = 'Traced';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the line vertices.  Each row is [nX nY].
%
%  PolyLine:
%     .strType = 'PolyLine';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the line vertices.  Each row is [nX nY].
%
%  FreeLine:
%     .strType = 'FreeLine';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the line vertices.  Each row is [nX nY].
%
%  Angle:
%     .strType = 'Angle';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the angle vertices.  Each row is [nX nY].
%
%  Point:
%     .strType = 'Point';
%     .mnCoordinates    - An [Nx2] matrix, specifying the coordinates of
%                          the points.  Each row is [nX nY].
%
%  NoROI:
%     .strType = 'NoROI';
%
% Additionally, ROIs from later versions (.nVersion >= 218) may have the
% following fields:
%
%     .nStrokeWidth     - The width of the line stroke
%     .nStrokeColor     - The encoded color of the stroke (ImageJ color format)
%     .nFillColor       - The encoded fill color for the ROI (ImageJ color
%                          format)
%
% If the ROI contains text:
%     .strSubtype = 'Text';
%     .nFontSize        - The desired font size
%     .nFontStyle       - The style of the font (unknown format)
%     .strFontName      - The name of the font to render the text with
%     .strText          - A string containing the text

% Author: Dylan Muir <dylan.muir@unibas.ch>
% Created: 9th August, 2011
%
% 20141020 Added code to read 'header 2' fields; thanks to Luca Nocetti
% 20140602 Bug report contributed by Samuel Barnes and Yousef Mazaheri
% 20110810 Bug report contributed by Jean-Yves Tinevez
% 20110829 Bug fix contributed by Benjamin Ricca <ricca@berkeley.edu>
% 20120622 Order of ROIs in a ROI set is now preserved
% 20120703 Different way of reading zip file contents guarantees that ROI order
%           is preserved
%
% Copyright (c) 2011, 2012, 2013, 2014 Dylan Muir <dylan.muir@unibas.ch>
%
% This program is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public License
% as published by the Free Software Foundation; either version 3
% of the License, or (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.

% -- Check arguments

if (nargin < 1)
   disp('*** ReadImageJROI: Incorrect usage');
   help ReadImageJROI;
   return;
end


% -- Check for a cell array of ROI filenames

if (iscell(cstrFilenames))
   % - Read each ROI in turn
   cvsROI = cellfun(@ReadImageJROI, CellFlatten(cstrFilenames), 'UniformOutput', false);
   
   % - Return all ROIs
   sROI = cvsROI;
   return;
   
else
   % - This is not a cell string
   strFilename = cstrFilenames;
   clear cstrFilenames;
end


% -- Check for a zip file

[nul, nul, strExt] = fileparts(strFilename); %#ok<ASGLU>
if (isequal(lower(strExt), '.zip'))
   % - get zp file contents
   cstrFilenames_short = listzipcontents_rois(strFilename);
   
   % - Unzip the file into a temporary directory
   strROIDir = tempname;
   unzip(strFilename, strROIDir);
   
   for (nFileIndex = 1:length(cstrFilenames_short))
      cstrFilenames{1, nFileIndex} = [strROIDir '/' char(cstrFilenames_short(nFileIndex, 1))];
   end
   
   % - Build ROIs for each file
   cvsROIs = ReadImageJROI(cstrFilenames);
   
   % - Clean up temporary directory
   delete([strROIDir filesep '*.roi']);
   rmdir(strROIDir);
   
   % - Return ROIs
   sROI = cvsROIs;
   return;
end


% -- Read ROI

% -- Check file and open
if (~exist(strFilename, 'file'))
   error('ReadImageJROI:FileNotFound', ...
      '*** ReadImageJROI: The file [%s] was not found.', strFilename);
end

fidROI = fopen(strFilename, 'r', 'ieee-be');

% -- Check file magic code
strMagic = fread(fidROI, [1 4], '*char');

if (~isequal(strMagic, 'Iout'))
   error('ReadImageJROI:FormatError', ...
      '*** ReadImageJROI: The file was not an ImageJ ROI format.');
end

% -- Read version
sROI.nVersion = fread(fidROI, 1, 'int16');

% -- Read ROI type
nTypeID = fread(fidROI, 1, 'uint8');
fseek(fidROI, 1, 'cof'); % Skip a byte

% -- Read rectangular bounds
sROI.vnRectBounds = fread(fidROI, [1 4], 'int16');

% -- Read number of coordinates
nNumCoords = fread(fidROI, 1, 'uint16');

% -- Read the rest of the header
vfLinePoints = fread(fidROI, 4, 'float32');
nStrokeWidth = fread(fidROI, 1, 'int16');
nShapeROISize = fread(fidROI, 1, 'uint32');
nStrokeColor = fread(fidROI, 1, 'uint32');
nFillColor = fread(fidROI, 1, 'uint32');
nROISubtype = fread(fidROI, 1, 'int16');
nOptions = fread(fidROI, 1, 'int16');
nArrowStyle = fread(fidROI, 1, 'uint8');
nArrowHeadSize = fread(fidROI, 1, 'uint8');
nRoundedRectArcSize = fread(fidROI, 1, 'int16');
sROI.nPosition = fread(fidROI, 1, 'uint32');


% -- Read the 'header 2' fields
nHeader2Offset = fread(fidROI, 1, 'uint32');

if (nHeader2Offset > 0) && ~fseek(fidROI, nHeader2Offset+32+4, 'bof') 
   % - Seek to start of header 2
   fseek(fidROI, nHeader2Offset+4, 'bof');
   
   % - Read fields
   sROI.vnPosition = fread(fidROI, 3, 'uint32')';
   vnNameParams = fread(fidROI, 2, 'uint32')';
   nOverlayLabelColor = fread(fidROI, 1, 'uint32'); %#ok<NASGU>
   nOverlayFontSize = fread(fidROI, 1, 'int16'); %#ok<NASGU>
   fseek(fidROI, 1, 'cof');   % Skip a byte
   nOpacity = fread(fidROI, 1, 'uint8'); %#ok<NASGU>
   nImageSize = fread(fidROI, 1, 'uint32'); %#ok<NASGU>
   fStrokeWidth = fread(fidROI, 1, 'float32'); %#ok<NASGU>
   vnROIPropertiesParams = fread(fidROI, 2, 'uint32')'; %#ok<NASGU>
   
else
   sROI.vnPosition = [];
   vnNameParams = [0 0];
   nOverlayLabelColor = []; %#ok<NASGU>
   nOverlayFontSize = []; %#ok<NASGU>
   nOpacity = []; %#ok<NASGU>
   nImageSize = []; %#ok<NASGU>
   fStrokeWidth = []; %#ok<NASGU>
   vnROIPropertiesParams = [0 0]; %#ok<NASGU>
end


% -- Set ROI name
if (isempty(vnNameParams) || any(vnNameParams == 0) || fseek(fidROI, sum(vnNameParams), 'bof'))
   [nul, sROI.strName] = fileparts(strFilename); %#ok<ASGLU>

else
   % - Try to read ROI name from header
   fseek(fidROI, vnNameParams(1), 'bof');
   sROI.strName = fread(fidROI, vnNameParams(2), 'int16=>char')';
end


% - Seek to get aspect ratio
fseek(fidROI, 52, 'bof');
fAspectRatio = fread(fidROI, 1, 'float32');

% - Seek to after header
fseek(fidROI, 64, 'bof');


% -- Build ROI

switch nTypeID
   case 1
      % - Rectangle
      sROI.strType = 'Rectangle';
      sROI.nArcSize = nRoundedRectArcSize;
      
      if (nShapeROISize > 0)
         % - This is a composite shape ROI
         sROI.strSubtype = 'Shape';
         if (nTypeID ~= 1)
            error('ReadImageJROI:FormatError', ...
               '*** ReadImageJROI: A composite ROI must be a Rectangle type.');
         end
         
         % - Read shapes
         sROI.vfShapes = fread(fidROI, nShapeROISize, 'float32');
      end
      
      
   case 2
      % - Oval
      sROI.strType = 'Oval';
      
   case 3
      % - Line
      sROI.strType = 'Line';
      sROI.vnLinePoints = round(vfLinePoints);
      
      if (nROISubtype == 2)
         % - This is an arrow line
         sROI.strSubtype = 'Arrow';
         sROI.bDoubleHeaded = nOptions & 2;
         sROI.bOutlined = nOptions & 4;
         sROI.nArrowStyle = nArrowStyle;
         sROI.nArrowHeadSize = nArrowHeadSize;
      end
      
      
   case 0
      % - Polygon
      sROI.strType = 'Polygon';
      sROI.mnCoordinates = read_coordinates;
      
   case 7
      % - Freehand
      sROI.strType = 'Freehand';
      sROI.mnCoordinates = read_coordinates;
      
      if (nROISubtype == 3)
         % - This is an ellipse
         sROI.strSubtype = 'Ellipse';
         sROI.vfEllipsePoints = vfLinePoints;
         sROI.fAspectRatio = fAspectRatio;
      end
      
   case 8
      % - Traced
      sROI.strType = 'Traced';
      sROI.mnCoordinates = read_coordinates;
      
   case 5
      % - PolyLine
      sROI.strType = 'PolyLine';
      sROI.mnCoordinates = read_coordinates;
      
   case 4
      % - FreeLine
      sROI.strType = 'FreeLine';
      sROI.mnCoordinates = read_coordinates;
      
   case 9
      % - Angle
      sROI.strType = 'Angle';
      sROI.mnCoordinates = read_coordinates;
      
   case 10
      % - Point
      sROI.strType = 'Point';
      sROI.mnCoordinates = read_coordinates;
      
   case 6
      sROI.strType = 'NoROI';
      
   otherwise
      error('ReadImageJROI:FormatError', ...
         '--- ReadImageJROI: The ROI file contains an unknown ROI type.');
end


% -- Handle version >= 218

if (sROI.nVersion >= 218)
   sROI.nStrokeWidth = nStrokeWidth;
   sROI.nStrokeColor = nStrokeColor;
   sROI.nFillColor = nFillColor;
   sROI.bSplineFit = nOptions & 1;
   
   if (nROISubtype == 1)
      % - This is a text ROI
      sROI.strSubtype = 'Text';
      
      % - Seek to after header
      fseek(fidROI, 64, 'bof');
      
      sROI.nFontSize = fread(fidROI, 1, 'uint32');
      sROI.nFontStyle = fread(fidROI, 1, 'uint32');
      nNameLength = fread(fidROI, 1, 'uint32');
      nTextLength = fread(fidROI, 1, 'uint32');
      
      % - Read font name
      sROI.strFontName = fread(fidROI, nNameLength, 'uint16=>char');
      
      % - Read text
      sROI.strText = fread(fidROI, nTextLength, 'uint16=>char');
   end
end

% - Close the file
fclose(fidROI);


% --- END of ReadImageJROI FUNCTION ---

   function [mnCoordinates] = read_coordinates
      % - Read X and Y coords
      vnX = fread(fidROI, [nNumCoords 1], 'int16');
      vnY = fread(fidROI, [nNumCoords 1], 'int16');
      
      % - Trim at zero
      vnX(vnX < 0) = 0;
      vnY(vnY < 0) = 0;
      
      % - Offset by top left ROI bound
      vnX = vnX + sROI.vnRectBounds(2);
      vnY = vnY + sROI.vnRectBounds(1);

      mnCoordinates = [vnX vnY];
   end

   function [filelist] = listzipcontents_rois(zipFilename)
      
      % listzipcontents_rois - FUNCTION Read the file names in a zip file
      %
      % Usage: [filelist] = listzipcontents_rois(zipFilename)
      
      % - Import java libraries
      import java.util.zip.*;
      import java.io.*;
      
      % - Read file list via JAVA object
      filelist={};
      in = ZipInputStream(FileInputStream(zipFilename));
      entry = in.getNextEntry();
      
      % - Filter ROI files
      while (entry~=0)
         name = entry.getName;
         if (name.endsWith('.roi'))
            filelist = cat(1,filelist,char(name));
         end;
         entry = in.getNextEntry();
      end;
      
      % - Close zip file
      in.close();
   end


   function [cellArray] = CellFlatten(varargin)
      
      % CellFlatten - FUNCTION Convert a list of items to a single level cell array
      %
      % Usage: [cellArray] = CellFlatten(arg1, arg2, ...)
      %
      % CellFlatten will convert a list of arguments into a single-level cell array.
      % If any argument is already a cell array, each cell will be concatenated to
      % 'cellArray' in a list.  The result of this function is a single-dimensioned
      % cell array containing a cell for each individual item passed to CellFlatten.
      % The order of cell elements in the argument list is guaranteed to be
      % preserved.
      %
      % This function is useful when dealing with variable-length argument lists,
      % each item of which can also be a cell array of items.
      
      % Author: Dylan Muir <dylan@ini.phys.ethz.ch>
      % Created: 14th May, 2004
      
      % -- Check arguments
      
      if (nargin == 0)
         disp('*** CellFlatten: Incorrect usage');
         help CellFlatten;
         return;
      end
      
      
      % -- Convert arguments
      
      if (iscell(varargin{1}))
         cellArray = CellFlatten(varargin{1}{:});
      else
         cellArray = varargin(1);
      end
      
      for (nIndexArg = 2:length(varargin)) %#ok<FORPF>
         if (iscell(varargin{nIndexArg}))
            cellReturn = CellFlatten(varargin{nIndexArg}{:});
            cellArray = [cellArray cellReturn{:}]; %#ok<AGROW>
         else
            cellArray = [cellArray varargin{nIndexArg}]; %#ok<AGROW>
         end
      end
      
      
      % --- END of CellFlatten.m ---
      
   end

end
function out=MoveCenterInfo(num,cells)
% Input: get data from data.mat, move boundary, convex_hull of 2date into center(0,0)
% Output: calculate distnace-distance, logD-logD

%  reveived values of: out.c1c2_cell, out.c1c2_hull, out.c1c2_distances,out.c1c2_logd
%example:   out=MoveCenter(2,cells);

out.c1_cell=[cells{1,num}.full_hulls{1,1}.x_cell(1,:);cells{1,num}.full_hulls{1,1}.y_cell(1,:)];
[out.c1_cell,center1]=movC(out.c1_cell,[0;0]);
out.c2_cell=[cells{1,num}.full_hulls{1,2}.x_cell(1,:);cells{1,num}.full_hulls{1,2}.y_cell(1,:)];
[out.c2_cell,center2]=movC(out.c2_cell,[0;0]);
out.c1_hull=[cells{1,num}.full_hulls{1,1}.x_hull(1,:);cells{1,num}.full_hulls{1,1}.y_hull(1,:)];
[out.c1_hull,center1]=movC(out.c1_hull,center1);
out.c2_hull=[cells{1,num}.full_hulls{1,2}.x_hull(1,:);cells{1,num}.full_hulls{1,2}.y_hull(1,:)];
[out.c2_hull,center2]=movC(out.c2_hull,center2);
dist_n=1000;
out.c1_distances=com_dist(cells{1,num}.full_hulls{1,1}.distances(1,:),dist_n);
out.c2_distances=com_dist(cells{1,num}.full_hulls{1,2}.distances(1,:),dist_n);
%out.c1_logd=log(com_dist(cells{1,num}.full_hulls{1,1}.distances(1,:),dist_n));
%out.c2_logd=log(com_dist(cells{1,num}.full_hulls{1,2}.distances(1,:),dist_n));
%out=center_dist(out);

    function [sk,sk2,sk3,xmi,ymi]=sketon(in)
        in=round(in);
        [a,b]=size(in);
        xma=max(in(1,:));xmi=min(in(1,:));
        yma=max(in(2,:));ymi=min(in(2,:));
        sk=zeros(xma-xmi+1,yma-ymi+1);
        for i=1:b
           sk(in(1,i)-xmi+1,in(2,i)-ymi+1)=1; 
        end
        for x=1:xma-xmi+1

        end
        sk3=bwmorph(sk,'bridge');
        sk2=bwmorph(sk3,'skel',inf);
    end
    function out=center_dist(out)
        c1_dist=zeros(1,size(out.c1_cell,2));c2_dist=zeros(1,size(out.c2_cell,2));
        for i=1:size(out.c1_cell,2)
            c1_dist(1,i)=sqrt(out.c1_cell(1,i)^2+out.c1_cell(2,i)^2);
        end
        for j=1:size(out.c2_cell,2)
            c2_dist(1,j)=sqrt(out.c2_cell(1,j)^2+out.c2_cell(2,j)^2);
        end
        out.c1_cdist=c1_dist;out.c2_cdist=c2_dist;
    end
end
function out=MoveCenterRun(out)
% make the cell boundary expansion 

for rate_s=2 %1.5:0.5:2.5
    eval(['out.expan.s3_s' num2str(rate_s) '=grow_s([out.c3.x_cell;out.c3.y_cell],' num2str(rate_s) ');']);
end
for rate_sb=4 %1.5:0.5:2.5
    sb2=grow_sb([out.c3.x_cell;out.c3.y_cell],rate_sb); %expand from boundary
    eval(['out.expan.s3_sb' num2str(rate_sb) '=short_move(sb2,' '50,' '0.1);']);
end

% Calculate Di, obj
out.expan.Di_1to2=compar2([out.c3.x_cell;out.c3.y_cell], [out.c6.x_cell;out.c6.y_cell]);
out.expan.Di_1to2=smooth(out.expan.Di_1to2);
out.expan.Di_1000=com_dist(out.expan.Di_1to2,1000);

    function [Di,Obj,Rdist]=compar(curve_origin, curve_obj,k,curve_originK)
        %sb2=grow_sb([out.c3.x_cell;out.c3.y_cell],rate_sb); %expand from boundary
        L0=length(curve_origin); L1=length(curve_obj);
        Di=zeros(1,L0);Obj=zeros(1,L0);Rdist=zeros(1,L0);
        %ra0(1,:)=curve_origin(2,:)./curve_origin(1,:);ra0=smooth(ra0);
        %ra1(1,:)=curve_obj(2,:)./curve_obj(1,:);ra1=smooth(ra1);
        ra0=Toangle(curve_origin);ra1=Toangle(curve_obj);     
        Obj=zeros(2,L0);
        for i=1:L0
            temp=ra1-ra0(1,i);
            %{
            for j=1:L1
                if curve_origin(1,i)*curve_obj(1,j)<=0&&curve_origin(2,i)*curve_obj(2,j)<=0
                    temp(1,j)=temp(1,j)+1000;
                end
            end
            %}
            [a,b]=min(abs(temp));
            Di(1,i)=sqrt(curve_obj(1,b)^2+curve_obj(1,b)^2)/sqrt(curve_origin(1,i)^2+curve_origin(1,i)^2); % Di: ratio of D_obj/D_origin   
            Rdist(1,i)=sqrt(curve_obj(1,b)^2+curve_obj(1,b)^2)-sqrt(curve_originK(1,i)^2+curve_originK(1,i)^2);
            dd=sqrt(curve_origin(1,i)^2+curve_origin(2,i)^2);
            Obj(1,i)=curve_origin(1,i)/dd*(dd+Di(1,i)*k);Obj(2,i)=curve_origin(2,i)/dd*(dd+Di(1,i)*k);
        end
        out.ra0=ra0;
        out.ra1=ra1;
    end
    function [Di]=compar2(curve_origin, curve_obj)
        rate_s_min=0;rate_s_max=4;rate_s_delta=0.01;
        L0=length(curve_origin); L1=length(curve_obj);
        Di=zeros(1,L0);
        for i=1:L0
            r0=100000;
            for rr=rate_s_min:rate_s_delta:rate_s_max
                mind=min_distance(curve_origin(:,i)*rr,curve_obj);
                if mind<r0
                    r0=mind;Di(i)=rr;
                end
            end
        end
    end
    function L2=smooth(L1)
        dL1=diff(L1);dL1_find=find(abs(dL1)>2);
        dstart=1;dend=0;L2=L1;
        for i=2:size(dL1_find,2)
            if(dL1_find(i)-dL1_find(i-1))>1
                dend=i-1;shift=min([dL1_find(dstart)-2, 20,size(L1,2)-dL1_find(dend)-2]);
                L2=smooth_single(L1,dL1_find(dstart)-shift,dL1_find(dend))+shift;
                dstart=i;
            end
            if i==size(dL1_find,2);
                dend=i;shift=min([dL1_find(dstart)-2, 20,size(L1,2)-dL1_find(dend)-2]);
                L2=smooth_single(L2,dL1_find(dstart)-shift,dL1_find(dend))+shift;
            end              
        end
    end
    function L2=smooth_single(L1,dstart,dend)
        L2=L1;
        for i=dstart:dend
            L2(1,i)=L1(1,dstart-1)+(L1(1,dend+1)-L1(1,dstart-1))*(i-dstart+1)/(dend-dstart+2);
        end
    end
    function ra=Toangle(curve)
        ra=zeros(1,size(curve,2));
        for i=1:size(curve,2)
            if curve(1,i)>=0&&curve(2,i)>=0
                ra(1,i)=atan(curve(2,i)/curve(1,i))/pi;
            elseif curve(1,i)<0
                ra(1,i)=atan(curve(2,i)/curve(1,i))/pi+1;
            elseif curve(1,i)>=0&&curve(2,i)<0
                ra(1,i)=atan(curve(2,i)/curve(1,i))/pi+2;
            end
        end
    end
    function mind=min_distance(point, vector)
        if size(point,1)~=2
            point=point';
        end
        if size(vector,1)~=2
            vector=vector';
        end
        mm=zeros(1,size(vector,2));
        mm=sqrt((vector(1,:)-point(1)).^2+(vector(2,:)-point(2)).^2);
        mind=min(mm);
    end
end
function cell = ProcessCell(point_before, p,cellname,replica)
% 1. Input cell(X,Y) and make cell convex hull.
% 2. Calculate curva and bending curve.

% Parameter deliver
cellname=p.cellname;
replica=p.replica;
adjust_hull='';
section_number=p.section_number;
d=p.d;
radius=p.radius;

% move to center and resize by r_x, r_y
[out2,ratio_r,center,first_point]=movC_resize(point_before(:,1:2)',zeros(2,1),p); 

% interpolation by spline2
points=spline2(out2',d,section_number);

% points corresponding junction points
points_row3=zeros(size(points,1),1);
if size(point_before,2)<3
    point_before=[point_before zeros(size(point_before,1),1)];
else    
    point_before_1=find(point_before(:,3)==1);point_before_1=point_before_1-first_point+1;
    for jj=1:size(point_before_1)
       if point_before_1(jj)>size(out2,2)
           point_before_1(jj)=point_before_1(jj)-size(out2,2);
       elseif point_before_1(jj)<1
           point_before_1(jj)=point_before_1(jj)+size(out2,2);
       end
    end
    A=out2(:,point_before_1);
    for ii=1: size(A,2)
        [~,a]=min(distance(A(:,ii)',points));
        points_row3(a,1)=1;
    end
end
points=[points points_row3];

% pre-setting points
if size(points,2)>3
    points=points';
end
%[points,center]=movC(points',zeros(2,1));points=points';  
x = points(:,1)';y = points(:,2)';xx=x;pointss=points;
x_o=x/ratio_r;y_o=y/ratio_r;

meanr=0;meanr_o=0;
for jj=1:size(x,2)-1
   meanr=meanr+sqrt(x(jj)^2+y(jj)^2);
   meanr_o=meanr_o+sqrt(x_o(jj)^2+y_o(jj)^2);
end
p.meanr=meanr/(size(x,2)-1);p.meanr_o=meanr_o/(size(x,2)-1);

k = convhull(x, y);
k=convhull_redirect(k);
cell = distanceFromHull(x, y, k, adjust_hull);
cell.distances1000=cell.distances1000/ratio_r/p.pixel_di_um;
cell.distances=cell.distances/ratio_r/p.pixel_di_um;
cell.xy = [x;y;points(:,3)'];cell.xy_o = [x_o;y_o;points(:,3)'];
cell.area=polyarea(cell.x_cell/ratio_r',cell.y_cell/ratio_r')/p.pixel_di_um^2;
cell.C_area=polyarea(cell.x_hull/ratio_r',cell.y_hull/ratio_r')/p.pixel_di_um^2;
%cell.perimeter=cal_dist([cell.x_cell/ratio_r;cell.y_cell/ratio_r]');
cell.perimeter=cal_dist([cell.xy(1,:)/ratio_r;cell.xy(2,:)/ratio_r]',p);
cell.C_perimeter=cal_dist([cell.x_hull/ratio_r;cell.y_hull/ratio_r]',p);
%cell.mean_radius=cal_radius([cell.x_cell/ratio_r;cell.y_cell/ratio_r]');
cell.mean_radius=radius/ratio_r/p.pixel_di_um;
[cell.klobe(:,1),cell.klobe(:,2)]=double_convhull(x,y,cell,p,k);x=xx;points=pointss;

cell2 = distanceFromHull(x, y, cell.klobe(:,1), adjust_hull);
cell.rev_distances1000=MA(com_dist(cell2.distances,1000))/ratio_r/p.pixel_di_um;
cell.rev_distancesReal=MA(com_dist(cell2.distances,round(cell.perimeter)*5))/ratio_r/p.pixel_di_um;
cell.rev_distances=cell2.distances/ratio_r/p.pixel_di_um;

[cell.klobe_e(:,1),cell.klobe_e(:,2)]=extract_lobe(cell.klobe,cell.xy);
[cell.klobe_e]=lob_match(cell.klobe_e,cell);
[cell.klobe2]=lob_match(cell.klobe,cell);
cell.klobe_num=max(cell.klobe(:,2));
cell.ratio_r=ratio_r;

% klobe_position
cell.klobe_position=zeros(3,cell.klobe_num);
for kk=1:cell.klobe_num
    xx=cell.xy_o(1,cell.klobe_e(kk,1));yy=cell.xy_o(2,cell.klobe_e(kk,1));
    if xx>=0&&yy>=0
        cell.klobe_position(1,kk)=atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif xx<0&&yy>=0
        cell.klobe_position(1,kk)=pi-atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif xx<0&&yy<0
        cell.klobe_position(1,kk)=pi+atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif  xx>=0&&yy<0
        cell.klobe_position(1,kk)=2*pi-atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    end
end

cell.info=showparameter(p,'c');
if any(strcmp('cen',fieldnames(p)))==1
    n_pp=size(p.cen,1);cexy=[x_o' y_o'];
    for iii=1:n_pp
        for iij=1:size(p.cen{iii,1},2)             
            p.ce{iii,1}(:,iij)=p.cen{iii,1}(1,iij);
        end
    end
end
if any(strcmp('ce',fieldnames(p)))==1
    n_pp=size(p.ce,1);cell.lobe_number_voting=zeros(1,n_pp);
    for pp=1:n_pp
        cell.lobe_number_voting(1,pp)=size(p.ce{pp,1},2);
    end
    [cell.Lb_Sensitivity, cell.Lb_Specificity, cell.Lb_Accuracy, cell.Lb_FDR, cell.lb_result]=lobe_position_result(cell.klobe_position(1,:),p.cen,size(point_before,1),p.correct_threa);
end
cell.compactness=4*pi*cell.area/(cell.perimeter)^2;
cell.roundness=4*pi*cell.area/(cell.C_perimeter)^2;
cell.convexity=cell.C_perimeter/cell.perimeter;
cell.solidity=cell.area/cell.C_area;
cell.p=p;

    function [k_I k_II] =  splitHull(x, y)
        % Find the control points for the convex hull and split it into the hulls
        % for each side.

        % find convex hull
        k = convhull(x, y);k=convhull_redirect(k);
        % split the hull into two sides
        k_I = [];
        k_II = [];
        k_last = -inf; %k(1);
        for k_i = k'
            if k_i > k_last
                k_I(end + 1) = k_i; 
            elseif k_i < k_last
                k_II(end + 1) = k_i;
            end
            k_last = k_i;
        end
        k_II = [k_I(end) k_II];
    end
    function data = distanceFromHull(x, y, k, adjust_hull)
        data = struct;

        %direction = 2*(k(2) > k(1))-1; % needed for the iterative part below
        %k = sort(k);

        data.arc = [];
        data.distances = [];
        data.x_cell = [];
        data.y_cell = [];
        data.x_hull = [];
        data.y_hull = [];
        d = 0.5;
        % d is the density of points sampled; 0.1 works well but may appear jagged

        % break the hull into bite-size pieces for analysis
        for f = 1:length(k)-1
            if isempty(data.arc) % arc_last tracks the last position along the hull so that lengths are additive
                arc_last = 0;
            else
                arc_last = data.arc(end);
            end

            k_1 = k(f);
            k_2 = k(f + 1);

            % create dense arrays of points on the hull
            hull_x = [];
            hull_y = [];
            hull_pos = [];
            distance = sqrt((x(k_1) - x(k_2)).^2 + (y(k_1) - y(k_2)).^2);
            for i = linspace(1, 2, ceil(distance/d))
                [hull_x(end + 1), hull_y(end + 1)] = interp_xy(x([k_1 k_2]), y([k_1 k_2]), i);
                if isempty(hull_pos)
                    hull_pos(end + 1) = 0;
                else
                    hull_pos(end + 1) = hull_pos(end) + sqrt((hull_x(end) - hull_x(end - 1)).^2 + (hull_y(end) - hull_y(end - 1)).^2 );
                end
            end

            % create dense arrays of points on the segment
            wall_x = [];
            wall_y = [];
            if k_2 > k_1
                for i = k_1:k_2-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
            else
                for i = k_1:length(x)-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
                for i = 1:k_2-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
                %disp(k_1)
                %disp(k_2)
                %disp('-----')
            end

            % find the closest point from a point on the hull to the segment
            for i = 1:length(wall_x)
                wx = wall_x(i);
                wy = wall_y(i);
                dist = inf;
                for j = 1:length(hull_x)
                    hx = hull_x(j);
                    hy = hull_y(j);
                    di = sqrt((wx - hx).^2 + (wy - hy).^2);
                    if di < dist
                        dist = di;
                        best = j;
                    end
                end
                if dist < inf
                    data.distances(end + 1) = dist; % record distance between points
                    data.arc(end + 1) = arc_last + hull_pos(best); % record distance along arc
                    data.x_cell(end + 1) = wx;
                    data.y_cell(end + 1) = wy;
                    data.x_hull(end + 1) = hull_x(best);
                    data.y_hull(end + 1) = hull_y(best);
                end
            end
        end
        
        %{
        [data.arc I] = sort(data.arc);
        data.distances = data.distances(I);
        data.x_cell = data.x_cell(I);
        data.y_cell = data.y_cell(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);

        [data.arc I] = remove_duplicates(data.arc, data.distances);
        data.distances = data.distances(I);
        data.x_cell = data.x_cell(I);
        data.y_cell = data.y_cell(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);
        %}
        %{
        if adjust_hull

            % Search for local minima (disregarding values < the sampling distance, d).
            % If any are found, clean out arc and distances, record the x/y coordinate,
            % insert a new control point for the "hull" and recalculate the
            % distance-from-hull for either side.  Then recombine the results.

            local_min = -1;
            value = inf;

            for i = 2:length(data.arc)-1
                if data.distances(i) > d && data.distances(i) < data.distances(i-1) && data.distances(i) < data.distances(i+1) && data.distances(i) < value
                    local_min = i;
                    value = data.distances(i);
                end
            end

            if local_min ~= -1 % split and re-solve...
                % We need to find the right place to insert the x,y coordinate into the
                % original x,y data
                x_new = data.x_cell(local_min);
                y_new = data.y_cell(local_min);
                for i = 1:length(x)-1
                    if ((x(i) <= x(i+1) && x(i) <= x_new && x_new <= x(i+1)) || (x(i) >= x(i+1) && x(i) >= x_new && x_new >= x(i+1))) && ...
                            ((y(i) <= y(i+1) && y(i) <= y_new && y_new <= y(i+1)) || (y(i) >= y(i+1) && y(i) >= y_new && y_new >= y(i+1)))
                        break
                    end
                end
                x_I = [x(1:i) x_new];
                x_II = [x_new x(i+1:end)];
                y_I = [y(1:i) y_new];
                y_II = [y_new y(i+1:end)];
                if length(x_I) >= 2 && length(x_II) >= 2 && (x_I(1) ~= x_I(2) || y_I(1) ~= y_I(2)) && (x_II(1) ~= x_II(2) || y_II(1) ~= y_II(2))
                    k_I = convhull(x_I, y_I);%k=convhull_redirect(k);
                    k_II = convhull(x_II, y_II);%k=convhull_redirect(k);
                    data_I = distanceFromHull(x_I, y_I, k_I, true);
                    data_II = distanceFromHull(x_II, y_II, k_II, true);

                    data.arc = [data_I.arc data_II.arc + data_I.arc(end)];
                    data.distances = [data_I.distances data_II.distances];
                    data.x_cell = [data_I.x_cell data_II.x_cell];
                    data.y_cell = [data_I.y_cell data_II.y_cell];
                    data.x_hull = [data_I.x_hull data_II.x_hull];
                    data.y_hull = [data_I.y_hull data_II.y_hull];

                    [data.arc I] = sort(data.arc);
                    data.distances = data.distances(I);
                    data.x_cell = data.x_cell(I);
                    data.y_cell = data.y_cell(I);
                    data.x_hull = data.x_hull(I);
                    data.y_hull = data.y_hull(I);

                    [data.arc I] = remove_duplicates(data.arc, data.distances);
                    data.distances = data.distances(I);
                    data.x_cell = data.x_cell(I);
                    data.y_cell = data.y_cell(I);
                    data.x_hull = data.x_hull(I);
                    data.y_hull = data.y_hull(I);
                end
            end
        end
%}
        cell_position = [0];
        for i = 2:length(data.x_cell)
            cell_position(end+1) = cell_position(end) + sqrt((data.x_cell(i) - data.x_cell(i-1))^2 + (data.y_cell(i) - data.y_cell(i-1))^2);
        end

        data.cell_position = cell_position;
        data.distances1000=MA(com_dist(data.distances,1000));
    end
    function [x, y] = interp_xy(X, Y, n)
        % Returns interpolation of the paired data X, Y for any n in the range
        % (integer or non-integer).
        if mod(n, 1) == 0
            x = X(n);
            y = Y(n);
        else
            n_1 = floor(n);
            n_2 = ceil(n);
            x = X(n_1) + (X(n_2) - X(n_1)) * mod(n, 1);
            y = Y(n_1) + (Y(n_2) - Y(n_1)) * mod(n, 1);
        end
    end
    function [x I] = remove_duplicates(x, y)
        % Remove duplicate 'x' entries.  Chooses the lowest "y".

        I = 1:length(x);

        dupe_found = true;
        i_last = 0;
        while dupe_found % this is an awful way to do this but it works fine
            dupe_found = false;
            for i_n = i_last+1:length(I)
                i = I(i_n);
                for j_n = i_n+1:length(I)
                    j = I(j_n);
                    if i ~= j && x(i) == x(j)
                        if y(i) < y(j)
                            I(j_n) = []; % delete the higher value
                        else
                            I(i_n) = [];
                        end
                        i_last = i;
                        dupe_found = true;
                        break
                    end
                    if dupe_found
                        break
                    end
                end
                if dupe_found
                    break
                end
            end
        end

        x = x(I);
    end
    function [newk,newkloop]=double_convhull(x,y,cell,p,k)
       if exist('k')==0
           k = convhull(x, y);k=convhull_redirect(k);
       end
       newkloop=convex2loop(x,y,k,p);
       [newk,newkloop]=second_convex(x,y,cell,k,newkloop,p);
    end
    function [newk,newkloop]=second_convex(x,y,cell,k,kloop,p)
        % second convex(search inner points)
        
        % use partial_convexhull to track newkk, and added to k2
        k2=k;newkk=[];
        for q=2:size(kloop,1)
            if abs(kloop(q)-kloop(q-1))>0
                newkk=partial_convhull(x,y,cell,k(q-1),k(q),p);
            end
            if isempty(newkk)==1
            else
                k2=[k2;newkk];
                newkk=[];
            end
        end
        
        % add new k2 into newk
        k3=[];
        sort(k2);k2=[k2;k2(1)];
        for i=1:size(x,2)
            if find(k2==i)
                k3=[k3;i];
            end
        end
        newk=[k3;k3(1)];
        
        % run convex2loop again to find new newkloop
        newkloop=convex2loop(x,y,newk,p);newkloop(newkloop==max(newkloop))=1;
    end
    function newk=partial_convhull(x,y,cell,startk,endk,p)
    [~,start_convd]=min((x(startk)-cell.x_cell).^2+(y(startk)-cell.y_cell).^2); % find the corresponding starting point on HODC distance from k 
    [~,end_convd]=min((x(endk)-cell.x_cell).^2+(y(endk)-cell.y_cell).^2); % find the corresponding end point on DOCH distance from k
    newk=[];
    if start_convd<end_convd
        Convex_dist=cell.distances(1,start_convd:end_convd); % specific region DOCH 
    elseif (start_convd>end_convd)&&(start_convd-end_convd<=0.5*size(cell.x_cell,2))
        Convex_dist=cell.distances(1,start_convd:-1:end_convd); % specific region DOCH(inverse direction)
    else
        Convex_dist=cell.distances(1,[start_convd:size(cell.distances,2) 1:end_convd]); % specific region DOCH across(0,0)
    end
    if distance(x(startk),y(startk),x(endk),y(endk))>p.gap_k | max(Convex_dist)/distance(x(startk),y(startk),x(endk),y(endk))>p.h_width_ratio
        varargout = peakfinder(Convex_dist,(max(Convex_dist)-min(Convex_dist))/p.sel,max(Convex_dist),-1); % find the local min points on Convex_dist
        %varargout=[varargout diff2(Convex_dist,p.diff2_n,p.diff2_thread)];
        [~,newk_n]=find(varargout~=1&varargout~=size(Convex_dist,2));
        if isempty(newk_n)~=1
            newk_xy=varargout(newk_n);nn=size(newk_xy,2);
            for i=1:nn
                if start_convd<end_convd  % specific region DOCH
                    [~,newkk]=min((cell.x_cell(newk_xy(i)+start_convd-1)-x).^2+(cell.y_cell(newk_xy(i)+start_convd-1)-y).^2);
                    newkkk=newkk;
                elseif (start_convd>end_convd)&&(start_convd-end_convd<=0.5*size(cell.x_cell,2))  % specific region DOCH(inverse direction)
                    [~,newkk]=min((cell.x_cell(start_convd-newk_xy(i)+1)-x).^2+(cell.y_cell(start_convd-newk_xy(i)+1)-y).^2);
                    newkkk=newkk;
            else  % specific region DOCH across(0,0)
                if newk_xy(i)<=(size(cell.x_cell,2)-start_convd)
                    [~,newkk]=min((cell.x_cell(newk_xy(i)+start_convd-1)-x).^2+(cell.y_cell(newk_xy(i)+start_convd-1)-y).^2);
                    newkkk=newkk;
                else
                    [~,newkk]=min((cell.x_cell(newk_xy(i)-size(cell.x_cell,2)+start_convd)-x).^2+(cell.y_cell(newk_xy(i)-size(cell.x_cell,2)+start_convd)-y).^2);
                    newkkk=newkk;
                end
            end
                newk=[newk;newkkk];
            end
        end
    end
    end
    function [mi_distance mi_p m2]=distance_p2line(line1,line2,points)
        % line1(start point)=(x1, y1), line2(end point)=(x2,y2)
        % points=[p1;p2;p3;p4.....];
        div=100;
        xspace=linspace(line1(1),line2(1),div);
        yspace=linspace(line1(2),line2(2),div);
        n=size(points,1);mi_distxy=zeros(n,1);
        for i=1:n
            mi_d=1000000;
            for j=1:div
            mi_d=min(mi_d,distance([xspace(j) yspace(j)],points(i,:)));
            end
            mi_distxy(i,1)=mi_d;
        end
        [mi_distance,m2]=max(mi_distxy);mi_p=points(m2,:);
    end
    function varargout = peakfinder(x0, sel, thresh, extrema)
        %PEAKFINDER Noise tolerant fast peak finding algorithm
        %   INPUTS:
        %       x0 - A real vector from the maxima will be found (required)
        %       sel - The amount above surrounding data for a peak to be
        %           identified (default = (max(x0)-min(x0))/4). Larger values mean
        %           the algorithm is more selective in finding peaks.
        %       thresh - A threshold value which peaks must be larger than to be
        %           maxima or smaller than to be minima.
        %       extrema - 1 if maxima are desired, -1 if minima are desired
        %           (default = maxima, 1)
        %   OUTPUTS:
        %       peakLoc - The indicies of the identified peaks in x0
        %       peakMag - The magnitude of the identified peaks
        %
        %   [peakLoc] = peakfinder(x0) returns the indicies of local maxima that
        %       are at least 1/4 the range of the data above surrounding data.
        %
        %   [peakLoc] = peakfinder(x0,sel) returns the indicies of local maxima
        %       that are at least sel above surrounding data.
        %
        %   [peakLoc] = peakfinder(x0,sel,thresh) returns the indicies of local 
        %       maxima that are at least sel above surrounding data and larger
        %       (smaller) than thresh if you are finding maxima (minima).
        %
        %   [peakLoc] = peakfinder(x0,sel,thresh,extrema) returns the maxima of the
        %       data if extrema > 0 and the minima of the data if extrema < 0
        %
        %   [peakLoc, peakMag] = peakfinder(x0,...) returns the indicies of the
        %       local maxima as well as the magnitudes of those maxima
        %
        %   If called with no output the identified maxima will be plotted along
        %       with the input data.
        %
        %   Note: If repeated values are found the first is identified as the peak
        %
        % Ex:
        % t = 0:.0001:10;
        % x = 12*sin(10*2*pi*t)-3*sin(.1*2*pi*t)+randn(1,numel(t));
        % x(1250:1255) = max(x);
        % peakfinder(x)
        %
        % Copyright Nathanael C. Yoder 2011 (nyoder@gmail.com)

        % Perform error checking and set defaults if not passed in
        error(nargchk(1,4,nargin,'struct'));
        error(nargoutchk(0,2,nargout,'struct'));

        s = size(x0);
        flipData =  s(1) < s(2);
        len0 = numel(x0);
        if len0 ~= s(1) && len0 ~= s(2)
            error('PEAKFINDER:Input','The input data must be a vector')
        elseif isempty(x0)
            varargout = {[],[]};
            return;
        end
        if ~isreal(x0)
            warning('PEAKFINDER:NotReal','Absolute value of data will be used')
            x0 = abs(x0);
        end

        if nargin < 2 || isempty(sel)
            sel = (max(x0)-min(x0))/4;
        elseif ~isnumeric(sel) || ~isreal(sel)
            sel = (max(x0)-min(x0))/4;
            warning('PEAKFINDER:InvalidSel',...
                'The selectivity must be a real scalar.  A selectivity of %.4g will be used',sel)
        elseif numel(sel) > 1
            warning('PEAKFINDER:InvalidSel',...
                'The selectivity must be a scalar.  The first selectivity value in the vector will be used.')
            sel = sel(1);
        end

        if nargin < 3 || isempty(thresh)
            thresh = [];
        elseif ~isnumeric(thresh) || ~isreal(thresh)
            thresh = [];
            warning('PEAKFINDER:InvalidThreshold',...
                'The threshold must be a real scalar. No threshold will be used.')
        elseif numel(thresh) > 1
            thresh = thresh(1);
            warning('PEAKFINDER:InvalidThreshold',...
                'The threshold must be a scalar.  The first threshold value in the vector will be used.')
        end

        if nargin < 4 || isempty(extrema)
            extrema = 1;
        else
            extrema = sign(extrema(1)); % Should only be 1 or -1 but make sure
            if extrema == 0
                error('PEAKFINDER:ZeroMaxima','Either 1 (for maxima) or -1 (for minima) must be input for extrema');
            end
        end

        x0 = extrema*x0(:); % Make it so we are finding maxima regardless
        thresh = thresh*extrema; % Adjust threshold according to extrema.
        dx0 = diff(x0); % Find derivative
        dx0(dx0 == 0) = -eps; % This is so we find the first of repeated values
        ind = find(dx0(1:end-1).*dx0(2:end) < 0)+1; % Find where the derivative changes sign

        % Include endpoints in potential peaks and valleys
        x = [x0(1);x0(ind);x0(end)];
        ind = [1;ind;len0];

        % x only has the peaks, valleys, and endpoints
        len = numel(x);
        minMag = min(x);


        if len > 2 % Function with peaks and valleys

            % Set initial parameters for loop
            tempMag = minMag;
            foundPeak = false;
            leftMin = minMag;

            % Deal with first point a little differently since tacked it on
            % Calculate the sign of the derivative since we taked the first point
            %  on it does not neccessarily alternate like the rest.
            signDx = sign(diff(x(1:3)));
            if signDx(1) <= 0 % The first point is larger or equal to the second
                ii = 0;
                if signDx(1) == signDx(2) % Want alternating signs
                    x(2) = [];
                    ind(2) = [];
                    len = len-1;
                end
            else % First point is smaller than the second
                ii = 1;
                if signDx(1) == signDx(2) % Want alternating signs
                    x(1) = [];
                    ind(1) = [];
                    len = len-1;
                end
            end

            % Preallocate max number of maxima
            maxPeaks = ceil(len/2);
            peakLoc = zeros(maxPeaks,1);
            peakMag = zeros(maxPeaks,1);
            cInd = 1;
            % Loop through extrema which should be peaks and then valleys
            while ii < len
                ii = ii+1; % This is a peak
                % Reset peak finding if we had a peak and the next peak is bigger
                %   than the last or the left min was small enough to reset.
                if foundPeak
                    tempMag = minMag;
                    foundPeak = false;
                end

                % Make sure we don't iterate past the length of our vector
                if ii == len
                    break; % We assign the last point differently out of the loop
                end

                % Found new peak that was lager than temp mag and selectivity larger
                %   than the minimum to its left.
                if x(ii) > tempMag && x(ii) > leftMin + sel
                    tempLoc = ii;
                    tempMag = x(ii);
                end

                ii = ii+1; % Move onto the valley
                % Come down at least sel from peak
                if ~foundPeak && tempMag > sel + x(ii)
                    foundPeak = true; % We have found a peak
                    leftMin = x(ii);
                    peakLoc(cInd) = tempLoc; % Add peak to index
                    peakMag(cInd) = tempMag;
                    cInd = cInd+1;
                elseif x(ii) < leftMin % New left minima
                    leftMin = x(ii);
                end
            end

            % Check end point
            if x(end) > tempMag && x(end) > leftMin + sel
                peakLoc(cInd) = len;
                peakMag(cInd) = x(end);
                cInd = cInd + 1;
            elseif ~foundPeak && tempMag > minMag % Check if we still need to add the last point
                peakLoc(cInd) = tempLoc;
                peakMag(cInd) = tempMag;
                cInd = cInd + 1;
            end

            % Create output
            peakInds = ind(peakLoc(1:cInd-1));
            peakMags = peakMag(1:cInd-1);
        else % This is a monotone function where an endpoint is the only peak
            [peakMags,xInd] = max(x);
            if peakMags > minMag + sel
                peakInds = ind(xInd);
            else
                peakMags = [];
                peakInds = [];
            end
        end

        % Apply threshold value.  Since always finding maxima it will always be
        %   larger than the thresh.
        if ~isempty(thresh)
            m = peakMags>thresh;
            peakInds = peakInds(m);
            peakMags = peakMags(m);
        end



        % Rotate data if needed
        if flipData
            peakMags = peakMags.';
            peakInds = peakInds.';
        end



        % Change sign of data if was finding minima
        if extrema < 0
            peakMags = -peakMags;
            x0 = -x0;
        end
        % Plot if no output desired
        if nargout == 0
            if isempty(peakInds)
                disp('No significant peaks found')
            else
                figure;
                plot(1:len0,x0,'.-',peakInds,peakMags,'ro','linewidth',2);
            end
        else
            varargout = {peakInds,peakMags};
        end
    end
    function k=diff2(in,n,thread)
        % to calculate second order differential value
        
        % outt = moving average(15) of in.
        k=[];
        nn=fix(size(in,2)/n);
        outt=zeros(1,nn);
        for i=1:nn
           outt(1,i)=mean(in((i-1)*n+1:i*n)); 
        end
        
        % obtain the second order difference of outt
        out=diff(outt,2); % 2nd diff
        [temp,b]=find(out>=thread);
        if isempty(b)~=1
            for j=1:size(b,2)
                [temp,bb]=max(diff(in(b(1,j)*n+1:(b(1,j)+1)*n),2));
                k=[k b(1,j)*n+1+bb];
            end
        end
    end
    function [klobe1,klobe2]=extract_lobe(klobe,xy)
        klobe1=zeros(max(klobe(:,2)),1);klobe2=zeros(max(klobe(:,2)),1);
        for i=1:max(klobe(:,2))
            se1=klobe(klobe(:,2)==i,1);
            if size(se1,1)==1
                klobe1(i)=klobe(klobe(:,2)==i,1);klobe2(i)=klobe(klobe(:,2)==i,2);
            elseif size(se1,1)>1 %&&i~=1
                if isempty(find(xy(3,se1(1):se1(end))==1, 1))~=1
                    [~,~, m2]=distance_p2line(xy(1:2,se1(1))',xy(1:2,se1(end))',xy(1:2,se1(1):se1(end))');
                    nn=find(xy(3,se1(1):se1(end))==1);
                    if abs(nn(1)-m2)<2
                        klobe1(i)=se1(1)+nn(1)-1;klobe2(i)=i;
                    else
                        klobe1(i)=m2+se1(1)-1;klobe2(i)=i;
                    end
                else
                    [mi_distance mi_p m2]=distance_p2line(xy(1:2,se1(1))',xy(1:2,se1(end))',xy(1:2,se1(1):se1(end))');
                    %[curva,~]=curvature(xy(1,se1(1):se1(end)),xy(2,se1(1):se1(end)),min(5,size(se1,1)));[~,m2]=max(curva);
                    klobe1(i)=m2+se1(1)-1;klobe2(i)=i;
                end
            elseif i==1
                temp=find(klobe(:,2)==i);aa=min(temp);
                klobe1(i)=aa; klobe2(i)=1;
            end
        end
    end
    function kloop=convex2loop(x,y,k,p)
        %if k(end)==k(1)
        %   k(end)=size(x,2); 
        %end
        kloop=zeros(size(k,1),size(k,2));
        loop_marker=1;kloop(1)=loop_marker;
        for i=2:size(k,1)
            if k(i)-k(i-1)>=0
                points=[x(k(i-1):k(i))' y(k(i-1):k(i))'];
            elseif k(i-1)-k(i)<size(x,2)*0.5
                points=[x(k(i-1):-1:k(i))' y(k(i-1):-1:k(i))'];
            else
                points=[x([k(i-1):end 1:k(i)])' y([k(i-1):end 1:k(i)])'];
            end
                mi_distance=distance_p2line([x(k(i-1)) y(k(i-1))],[x(k(i)) y(k(i))],points);   
            if distance(x(k(i)),y(k(i)),x(k(i-1)),y(k(i-1)))>p.gap_k|mi_distance/distance([x(k(i-1)) y(k(i-1))],[x(k(i)) y(k(i))])>p.h_width_ratio
                    loop_marker=loop_marker+1;
            end
            kloop(i)=loop_marker;
        end
    end
    function newk=convhull_redirect(k)
        ord=diff(k);[a,~]=size(k);
        posi=numel(find(ord>0));negi=numel(find(ord<0));
        if posi>negi
            newk=k;
        elseif posi<negi
            newk=zeros(a,1);
            for i=1:a
                newk(i,1)=k(a-i+1,1);
            end
        end
    end
    function [klobe_e]=lob_match(klobe_e,cell)
        n2=size(cell.x_cell,2);
        n3=size(cell.rev_distancesReal,2);n0=size(klobe_e,1);
        klobe_e1=zeros(n0,1);klobe_e2=zeros(n0,1);
        for i=1:n0
            x0=cell.xy(1,klobe_e(i,1));y0=cell.xy(2,klobe_e(i,1));
            n2_dist=zeros(1,n2);n3_dist=zeros(1,n3);
            for k1=1:n2
                n2_dist(1,k1)=(x0-cell.x_cell(k1))^2+(y0-cell.y_cell(k1))^2;
            end
            [~,klobe_e1(i,1)]=min(n2_dist);
            klobe_e2(i,1)=round(n3/n2*klobe_e1(i,1));
            klobe_e2(i,1)=zero_fix(klobe_e2(i,1),cell.rev_distancesReal);
        end
        klobe_e=[klobe_e(:,1) klobe_e1 klobe_e2 klobe_e(:,2)];
    end
end
function e2=zero_fix(e2,data)
if data(e2)==0
else
n=5;stop=0;
while stop==0&&n<30
    e_start=max(1,e2-n);e_end=max(1,e2+n);data_s=data(1,e_start:e_end);
    Zero_position=find(data_s==0);
    if isempty(Zero_position)==1
        n=n+5;
    else
        [~,m]=min(abs(Zero_position+e_start-e2-1));
        e2=e_start+Zero_position(m)-1;stop=1;
    end
end
end
end
function [out5,lop_range] = blankcell(xyt0,lop,contP)
% make the blank cell boundary from (t0). with designed parameter.
% xyt0: input XY(to) (2,n)
% expan_model: 1= expansion from center, 2=expansion from boundary (1 or 2)
% r: expansion rate (1+k)
% fuz: given noisy (25:30)
% lop_xy: designed loop center position
% lop_p: designed loop standard deviation ()
% lop_height: designed loop height ()
% example:  out = blankcell(xyt0, 1,r, fuz, lop);
lop.contP=contP;
lop=createloop(lop);
if size(xyt0,1)>2
    xyt0=xyt0';
end
    
[out2,center]=movC(xyt0,[0;0]);
if lop.expan_model==1
    out3=grow_s(out2,lop.r);
elseif lop.expan_model==2
    out3=grow_sb(out2,lop.r);
    out3=short_move(out3,30,0.5);
end
    [out4,lop_range]=grow_loop(out3,lop.lop_xy,lop.lop_p,lop.lop_height);
    if lop.noise_r==0
        out5=out4;
    elseif lop.noise_r==1
        out5=grow_noisy(out4,lop.contP,lop.noise_p,lop.noise_height);
    end
end
function segment=three_junctions(t1,t2,t1s,t2s)
    segment=[];
    segment=junction_time(segment,t1,t1s,'t1');
    segment=junction_time(segment,t2,t2s,'t2');
    segment.t1t2_jun_matrix=segment.t2_jun_matrix./segment.t1_jun_matrix;
    segment.t1t2_jun_matrix(isnan(segment.t1t2_jun_matrix))=0;

    function seg=junction_time(seg,t,ts,tim)
        points=t.xy';
        num=find(points(:,3)==1);num=num(1:end-1);jun_num=size(num,1);
        se.point=[points(num,1:2) num]';
        temp_segp=[se.point [se.point(1:2,1);size(points,1)]];
        temp_points=points;
        jundistance=zeros(4,jun_num);lobe=zeros(2,jun_num);
        for i=1:jun_num
            jundistance(1,i)=0;
            for j=temp_segp(3,i):temp_segp(3,i+1)-1
                jundistance(1,i)=jundistance(1,i)+distance(temp_points(j,1),temp_points(j,2),temp_points(j+1,1),temp_points(j+1,2));
            end
            jundistance(2,i)=distance(temp_segp(1,i),temp_segp(2,i),temp_segp(1,i+1),temp_segp(2,i+1));
            jundistance(3,i)=jundistance(1,i)/jundistance(2,i);
            lobe_num=find(t.klobe_e(:,1)>temp_segp(3,i) & t.klobe_e(:,1)<temp_segp(3,i+1));
            lobe_num_b=find(t.klobe_e(:,1)==temp_segp(3,i));
            if isempty(lobe_num)~=1
                lobe(1,i)=size(lobe_num,1);
            end
            if isempty(lobe_num_b)~=1
                lobe(2,i)=size(lobe_num_b,1);
            end
        end
        jun_matrix=zeros(jun_num,jun_num);
        for i=1:jun_num
            jundistance(4,i)=jundistance(1,i)/sum(jundistance(1,:));
            for j=1:jun_num
                jun_matrix(i,j)=distance(se.point(1,i),se.point(2,i),se.point(1,j),se.point(2,j));
            end
        end
        eval(['seg.' tim '_jun_point=[points(num,1:2) num];']);
        eval(['seg.' tim '_jun_dist=jundistance;']);
        eval(['seg.' tim '_jun_matrix=jun_matrix;']);
        eval(['seg.' tim '_lobe=lobe;']);
    end
end
function seg=junction_time(t)
% get junction and junction section properties     
        points=t.xy_o';
        num=find(points(:,3)==1);%num=num(1:end-1);
        jun_num=size(num,1);
        se.point=[points(num,1:2) num]';  % se.point: junction points (x,y)
        temp_segp=[se.point [se.point(1:2,1);size(points,1)]];
        temp_points=points;
        jundistance=zeros(4,jun_num);lobe=zeros(2,jun_num);
        seg.results=zeros(jun_num,13);
        p_tot=0;a_tot=0;
        seg.lobe_section_position{1,jun_num}=[];
        
        % get junction points on x_cell,x_hull
        hull_n=zeros(1,jun_num+1);
        for i=1:jun_num
            [~,hull_n(1,i)]=min(distance([t.x_cell;t.y_cell]',t.xy(1:2,num(i,1))'));
        end
        hull_n(1,jun_num+1)=size(t.x_hull,2);
        
        
        for i=1:jun_num
            if i~=jun_num
                seg.section{i}.xy=points(num(i):num(i+1),:);
            else
                seg.section{i}.xy=points(num(i):end,:);
            end
            hull_points=[(t.x_hull(hull_n(i):hull_n(i+1))/t.ratio_r)' (t.y_hull(hull_n(i):hull_n(i+1))/t.ratio_r)'];
            cell_points=[(t.x_cell(hull_n(i):hull_n(i+1))/t.ratio_r)' (t.y_cell(hull_n(i):hull_n(i+1))/t.ratio_r)'];
            seg.section{i}.area=polyarea([seg.section{i}.xy(:,1);0],[seg.section{i}.xy(:,2);0]);
            seg.section{i}.C_area=polyarea([hull_points(:,1);0],[hull_points(:,2);0]);
            seg.section{i}.perimeter=cal_dist([seg.section{i}.xy(:,1),seg.section{i}.xy(:,2)]);
            seg.section{i}.C_perimeter=cal_dist([hull_points(:,1),hull_points(:,2)]);
            seg.section{i}.mean_radius=cal_radius([seg.section{i}.xy(:,1),seg.section{i}.xy(:,2)]);
            
            seg.section{i}.compactness=2*seg.section{i}.area/(seg.section{i}.perimeter*seg.section{i}.mean_radius);
            seg.section{i}.roundness=2*seg.section{i}.area/(seg.section{i}.C_perimeter*seg.section{i}.mean_radius);
            seg.section{i}.convexity=seg.section{i}.C_perimeter/seg.section{i}.perimeter;
            seg.section{i}.solidity=seg.section{i}.area/seg.section{i}.C_area;
            
            p_tot=p_tot+seg.section{i}.perimeter;
            a_tot=a_tot+seg.section{i}.area;
            jundistance(1,i)=0;
            for j=temp_segp(3,i):temp_segp(3,i+1)-1
                jundistance(1,i)=jundistance(1,i)+distance(temp_points(j,1),temp_points(j,2),temp_points(j+1,1),temp_points(j+1,2));
            end
            jundistance(2,i)=cal_dist([hull_points(:,1),hull_points(:,2)]);
            jundistance(3,i)=jundistance(1,i)/jundistance(2,i);
            lobe_num=find(t.klobe_e(:,1)>temp_segp(3,i) & t.klobe_e(:,1)<temp_segp(3,i+1));
            lobe_num_b=find(t.klobe_e(:,1)==temp_segp(3,i));
            if isempty(lobe_num)~=1
                lobe(1,i)=size(lobe_num,1);
                for kk=1:lobe(1,i)
                    [~,bb]=min(distance([seg.section{i}.xy(:,1),seg.section{i}.xy(:,2)],t.klobe_position(2:3,lobe_num(kk))'));
                    bbb=cal_dist([seg.section{i}.xy(1:bb,1),seg.section{i}.xy(1:bb,2)])/seg.section{i}.perimeter;
                    seg.lobe_section_position{1,i}=[bbb seg.lobe_section_position{1,i}];
                end
            end
            if isempty(lobe_num_b)~=1
                lobe(2,i)=size(lobe_num_b,1);
            end           
            seg.results(i,1:2)=[lobe(1,i) lobe(2,i)];
            seg.results(i,3)=seg.section{i}.area;
            seg.results(i,4)=seg.section{i}.C_area;
            seg.results(i,5)=seg.section{i}.perimeter;
            seg.results(i,6)=seg.section{i}.C_perimeter;
            seg.results(i,7)=seg.section{i}.mean_radius;
            seg.results(i,8)=seg.section{i}.compactness;
            seg.results(i,9)=seg.section{i}.roundness;
            seg.results(i,10)=seg.section{i}.convexity;
            seg.results(i,11)=seg.section{i}.solidity;
        end
        for i=1:jun_num
            seg.section{i}.area_ratio=seg.section{i}.area/a_tot;
            seg.section{i}.perimeter_ratio=seg.section{i}.perimeter/p_tot;
            seg.results(i,12)=seg.section{i}.perimeter_ratio;
            seg.results(i,13)=seg.section{i}.area_ratio;
        end
        jun_matrix=zeros(jun_num,jun_num);
        for i=1:jun_num
            jundistance(4,i)=jundistance(1,i)/sum(jundistance(1,:));
            for j=1:jun_num
                jun_matrix(i,j)=distance(se.point(1,i),se.point(2,i),se.point(1,j),se.point(2,j));
            end
        end
        seg.jun_point=[points(num,1:2) num];
        seg.jun_dist=jundistance;
        seg.jun_matrix=jun_matrix;
        seg.lobe=lobe;
        seg.info_results={'lobe' 'lobe&junction' 'area' 'C_area' 'perimeter' 'C_perimeter' 'mean_radius' 'compactness' 'roundness' 'convexity' 'solidity' 'p_ratio' 'a_ratio'};
end
function [global_m, seg_m, info]=do_result_matrix(note,t1,t2,t1Jresult,t2Jresult)
    n=size(t1Jresult,1);
    t1_total_info=[sum(t1Jresult(:,1)) sum(t1Jresult(:,2)) t1.area ...
        t1.C_area t1.perimeter t1.C_perimeter t1.mean_radius ...
        t1.compactness t1.roundness t1.convexity t1.solidity];
    t2_total_info=[sum(t2Jresult(:,1)) sum(t2Jresult(:,2)) t2.area ...
        t2.C_area t2.perimeter t2.C_perimeter t2.mean_radius ...
        t2.compactness t2.roundness t2.convexity t2.solidity];
    total_info_change=[t2_total_info(1:2)-t1_total_info(1:2) t2_total_info(3:7)./t1_total_info(3:7) t2_total_info(8:11)-t1_total_info(8:11)];
    jresult_change=[t2Jresult(:,1:2)-t1Jresult(:,1:2) t2Jresult(:,3:7)./t1Jresult(:,3:7) t2Jresult(:,8:13)-t1Jresult(:,8:13)];
    global_m=[note 0 t1_total_info t2_total_info total_info_change];
    seg_m=[repmat(note,n,1) (1:n)' t1Jresult t2Jresult jresult_change];
    info.global={'dataset' 'cell' 'replicate' 't1' 't2' '0' 'lobe' 'lobe_jun' 'area' 'C_area' 'perimeter' 'C_perimeter' 'mean_radius' 'compactness' 'roundness' 'convexity' 'solidity'};
    info.seg={'dataset' 'cell' 'replicate' 't1' 't2' 'seg_no' 'lobe' 'lobe_jun' 'area' 'C_area' 'perimeter' 'C_perimeter' 'mean_radius' 'compactness' 'roundness' 'convexity' 'solidity' 'p_ratio' 'a_ratio'};
end
function [output1,d_direct,d_boundary]=spline2(input,d,section_number)
% ex:   [output1,d_direct,d_boundary]=spline2(xy,1,10);
% out: (nx2), in: (mx2), d: distance between points

n=size(input,1);D=diff(input);total_distance=0;origin=0;

% setting d from d or section_numbere
if d==0&&section_number~=0
    for i=2:n
       total_distance=total_distance+distance(input(i-1,:),input(i,:)); 
    end
    d=total_distance/section_number;
elseif d==0&&section_number==0
    origin=1;
end
section_head0=1;section_head=1;output2=[];out_list=1;d_direct=[];d_boundary=[];

% get section_head
for i=1:n-2
    if D(i,1)*D(i+1,1)<=0||D(i,2)*D(i+1,2)<=0
        section_head0=[section_head0;i+1];
    end
end
for j=2:size(section_head0,1)-1
    if abs(section_head0(j-1)-section_head0(j))~=1||abs(section_head0(j+1)-section_head0(j))~=1
        section_head=[section_head;section_head0(j)];
    end
end
section_head=[section_head;n];
d_section_head=diff(section_head);
short_section=find(d_section_head<1);
section_head2=[];

% get section_head2
if isempty(short_section)==1
    section_head2=section_head;
else
    for i=1:size(section_head,1)
        if isempty(find(short_section==i))==1&&isempty(find(short_section==i-1))==1
        section_head2=[section_head2;section_head(i,1)];
        end
    end
end

% run spline3 for each section, and obtain output2 
%section_head=section_head2;
section_head=(1:n)';
for j=1:size(section_head,1)-1
    output2=[output2;spline3(input(section_head(j):section_head(j+1),:))]; % section2 applied spline3
end
dd=0;ddd=0;

% cut the output2 series as interporation d series -->out_list = section head number 
for i=2:size(output2,1)
    dd=dd+distance(output2(i-1,1),output2(i-1,2),output2(i,1),output2(i,2));
    ddd=dd+distance(output2(out_list(end,1),1),output2(out_list(end,1),2),output2(i,1),output2(i,2));
    if ddd>=d
        out_list=[out_list;i];
        d_direct=[d_direct;ddd];d_boundary=[d_boundary;dd];
        ddd=0;dd=0;
    end
end
if out_list(end,1)~=size(output2,1)
    out_list=[out_list;size(output2,1)];
end
% obtain the output1 from out_list
output1=output2(out_list,:);
if origin==1;
    output1=input;
end
function output1=spline3(input)
    in_d=0.1;
    if abs(input(1,1)-input(end,1))>=abs(input(1,2)-input(end,2))
        if input(1,1)<=input(end,1)
            xx=input(1,1):in_d:input(end,1);
        else
            xx=input(1,1):-in_d:input(end,1);
        end
        yy=spline(input(:,1),input(:,2),xx);
        output1=[xx' yy'];
    else
        if input(1,2)<=input(end,2)
            xx=input(1,2):in_d:input(end,2);
        else
            xx=input(1,2):-in_d:input(end,2);
        end
        yy=spline(input(:,2),input(:,1),xx);
        output1=[yy' xx'];
    end     
end 
end
function [Xmap,Ymap,xy_value_map2]=make_image_from_line(xy,xy_value,CorB)
% [xy_value_map2]=make_image_from_line(out.xy,out.curva);

dx_map=0.9;
dy_map=0.9;
lis_num=50; %linspace numer=50
x_min=min(xy(:,1));x_max=max(xy(:,1));
y_min=min(xy(:,2));y_max=max(xy(:,2));
x_axis=x_min-(x_max-x_min)*0.2:dx_map:x_max+(x_max-x_min)*0.2;
y_axis=y_min-(y_max-y_min)*0.2:dy_map:y_max+(y_max-y_min)*0.2;
[Xmap,Ymap]=meshgrid(x_axis,y_axis);
xy_value_map=value_map(x_axis,y_axis,xy(:,1:2),xy_value,lis_num);
if CorB==1
    Cmap1=jet(125);Cmap1(61,:)=1;
    xy_value_map2=xy_value_map;xy_value_map2(xy_value_map==0)=0;
    
    fig1 = figure;
    surf(Xmap,Ymap,xy_value_map,xy_value_map2,'FaceColor','interp','EdgeColor','none');
    Cmap=colormap(Cmap1);
    set(gca,'YDir','reverse');
    axis off
    view(0,92);
    material shiny;
    title('Curvature Map','Color','k','FontSize',12,'FontWeight','demi');
    colorbar;
else
    Cmap1=jet(125);Cmap1(1,:)=1;
    xy_value_map2=xy_value_map;xy_value_map2(xy_value_map==0)=0;
    
    fig1 = figure;
    surf(Xmap,Ymap,xy_value_map,xy_value_map2,'FaceColor','interp','EdgeColor','none');
    Cmap=colormap(Cmap1);
    set(gca,'YDir','reverse');
    axis off
    view(0,92);
    material shiny;
    title('Bending Energy Map','Color','k','FontSize',12,'FontWeight','demi');
    colorbar;
end
    function xy_value_map=value_map(x_axis,y_axis,xy,xy_value,lis_num)
        xy_value_map=zeros(size(y_axis,2),size(x_axis,2));
        n=size(xy,1);
        for i=2:n
            x_series=linspace(xy(i-1,1),xy(i,1),lis_num);
            y_series=linspace(xy(i-1,2),xy(i,2),lis_num);
            value_series=linspace(xy_value(1,i-1),xy_value(1,i),lis_num);
            for j=1:lis_num
                [~,nx]=min(abs(x_axis-x_series(j)));
                [~,ny]=min(abs(y_axis-y_series(j)));
                xy_value_map(ny,nx)=value_series(j);
                clear nx ny
            end
            clear x_series y_series
        end
    end
end
function ce=synthesis_data(r1,r2,lobe_n,all_n,r_xy)
% make synthesis pavement cell boundary dataset
%example: synthesis_data(20,18,10,10);

% setting
ce={};  %output variable {nx2}
fielname='datas_1.mat';  %output mat name  

% -----------------------------------------------
for kkk=1:all_n
    [out,lobe_position]=create_paveC(r1,r2,lobe_n,r_xy);
    %eval(['ce' num2str(kkk) 'a.t1=out;']);
    %eval(['ce' num2str(kkk) 'a.t1_lobeP=lobe_position;']);
    %eval(['assignin(''base'', ''ce' num2str(kkk) 'a'', out);']);
    %eval(['assignin(''base'', ''ce' num2str(kkk) 'alobe'', lobe_position);']);
    eval(['ce{1,' num2str(kkk) '}=out;']);
    eval(['ce{2,' num2str(kkk) '}=lobe_position;']);
end
%clear r1 r2 lobe_n all_n kkk out lobe_position
%save(fielname,'ce');

    function [out,lobe_position]=create_paveC(r1,r2,lobe_n,r_xy)
        % create some synthesis pavement cell boundary dataset.
        % example: [out, lobe_position]=create_paveC(20,18,10);

        %lobe_n: designed lobe numer
        %r1: outter circle radius
        %r2: inner circle radius
        %out: Synthesis pavement cell boundary xy(2xn) 

        %--------------------------------------------
        %setting
        data_n=100;
        inner_probability=0.2;

        %choice outter point
        m_d_lobe_order=0;lobe_position=zeros(3,lobe_n);lobe_position2=zeros(1,lobe_n);
        while m_d_lobe_order<5
            lobe_order=[1 sort(round(rand(1,lobe_n-1)*data_n)) data_n];
            m_d_lobe_order=min(diff(lobe_order));
        end

        %choice inner point
        inner_order=[];out2=[];
        for j=1:lobe_n
            lobe_position(1,j)=lobe_order(j)/50*pi;
            lobe_position(2,j)=r1*cos(lobe_position(1,j));
            lobe_position(3,j)=r1*sin(lobe_position(1,j));
            nn=0;
            out2=[out2 [lobe_order(j);r1*cos(lobe_order(j)/50*pi);r1*sin(lobe_order(j)/50*pi)]];
           for k=lobe_order(j)+2:lobe_order(j+1)-2
               if k==lobe_order(j)+2
                   rr2=rand(1,1)*(r1-r2-3)+r2;
                   %rr2=r2;
               end
               if rand(1,1)<inner_probability
                   inner_order=[inner_order k];
                   out2=[out2 [k;rr2*cos(k/50*pi);rr2*sin(k/50*pi)]];
                   nn=nn+1;
               end
               if k==lobe_order(j+1)-2&&nn==0
                   inner_order=[inner_order round((lobe_order(j)+lobe_order(j+1))/2)];
                   kk=round((lobe_order(j)+lobe_order(j+1))/2);
                   out2=[out2 [kk;rr2*cos(kk/50*pi);rr2*sin(kk/50*pi)]];
               end
           end
        end
        %r_xy=rand(1,2)*3+1;
        %r_xy=[2 1];
        out2=[out2 out2(:,1)];
        out2=[out2(2,:)*r_xy(1,1);out2(3,:)*r_xy(1,2)]';
        for j=1:lobe_n
            lobe_position(1,j)=expan_thi(lobe_position(1,j),r_xy(1,1),r_xy(1,2));
            lobe_position(2,j)=lobe_position(2,j)*r_xy(1,1);
            lobe_position(3,j)=lobe_position(3,j)*r_xy(1,2);
        end
        %create output dataset
        out=spline2(out2(:,1:2),0,40);
        
    end
    function [output1,d_direct,d_boundary]=spline2(input,d,section_number) 
    % ex:   [output1,d_direct,d_boundary]=spline2(xy,1,10);
    % out: (nx2), in: (mx2), d: distance between points

    n=size(input,1);D=diff(input);total_distance=0;origin=0;

    % setting d from d or section_numbere
    if d==0&&section_number~=0
        for i=2:n
           total_distance=total_distance+distance(input(i-1,:),input(i,:)); 
        end
        d=total_distance/section_number;
    elseif d==0&&section_number==0
        origin=1;
    end
    section_head0=1;section_head=1;output2=[];out_list=1;d_direct=[];d_boundary=[];

    % get section_head
    for i=1:n-2
        if D(i,1)*D(i+1,1)<=0||D(i,2)*D(i+1,2)<=0
            section_head0=[section_head0;i+1];
        end
    end
    for j=2:size(section_head0,1)-1
        if abs(section_head0(j-1)-section_head0(j))~=1||abs(section_head0(j+1)-section_head0(j))~=1
            section_head=[section_head;section_head0(j)];
        end
    end
    section_head=[section_head;n];
    d_section_head=diff(section_head);
    short_section=find(d_section_head<1);
    section_head2=[];

    % get section_head2
    if isempty(short_section)==1
        section_head2=section_head;
    else
        for i=1:size(section_head,1)
            if isempty(find(short_section==i))==1&&isempty(find(short_section==i-1))==1
            section_head2=[section_head2;section_head(i,1)];
            end
        end
    end

    % run spline3 for each section, and obtain output2 
    %section_head=section_head2;
    section_head=(1:n)';
    for j=1:size(section_head,1)-1
        output2=[output2;spline3(input(section_head(j):section_head(j+1),:))]; % section2 applied spline3
    end
    dd=0;ddd=0;

    % cut the output2 series as interporation d series -->out_list = section head number 
    for i=2:size(output2,1)
        dd=dd+distance(output2(i-1,1),output2(i-1,2),output2(i,1),output2(i,2));
        ddd=dd+distance(output2(out_list(end,1),1),output2(out_list(end,1),2),output2(i,1),output2(i,2));
        if ddd>=d
            out_list=[out_list;i];
            d_direct=[d_direct;ddd];d_boundary=[d_boundary;dd];
            ddd=0;dd=0;
        end
    end
    if out_list(end,1)~=size(output2,1)
        out_list=[out_list;size(output2,1)];
    end
    % obtain the output1 from out_list
    output1=output2(out_list,:);
    if origin==1;
        output1=input;
    end
    function output1=spline3(input)
        in_d=0.1;
        if abs(input(1,1)-input(end,1))>=abs(input(1,2)-input(end,2))
            if input(1,1)<=input(end,1)
                xx=input(1,1):in_d:input(end,1);
            else
                xx=input(1,1):-in_d:input(end,1);
            end
            yy=spline(input(:,1),input(:,2),xx);
            output1=[xx' yy'];
        else
            if input(1,2)<=input(end,2)
                xx=input(1,2):in_d:input(end,2);
            else
                xx=input(1,2):-in_d:input(end,2);
            end
            yy=spline(input(:,2),input(:,1),xx);
            output1=[yy' xx'];
        end     
    end 
    end
    function thi2=expan_thi(thi,r_x,r_y)
    % thi (0-2pi) change after rxpansion xy
        if 0<=thi&&thi<0.5*pi
            thi2=atan(tan(thi)*r_y/r_x);
        elseif 0.5*pi<=thi&&thi<pi
            thi2=pi-atan(tan(pi-thi)*r_y/r_x);
        elseif pi<=thi&&thi<1.5*pi
           thi2=atan(tan(thi-pi)*r_y/r_x)+pi; 
        elseif 1.5*pi<=thi&&thi<2*pi
            thi2=2*pi-atan(tan(2*pi-thi)*r_y/r_x);
        end
    end
end
function Save_excel(p,L_xy,L_name,Lobe_result,excel_name)

N_name=p.excel_name;
he=actxserver('Excel.Application');
excelVersion = str2double(he.Version);
D=cd;
if excelVersion < 12
	excelExtension = '.xls';
else
	excelExtension = '.xlsx';
end
switch excelExtension
	case '.xls' %xlExcel8 or xlWorkbookNormal
	   xlFormat = -4143;
	case '.xlsb' %xlExcel12
	   xlFormat = 50;
	case '.xlsx' %xlOpenXMLWorkbook
	   xlFormat = 51;
	case '.xlsm' %xlOpenXMLWorkbookMacroEnabled 
	   xlFormat = 52;
	otherwise
	   xlFormat = -4143;
end
N=[N_name excelExtension];
eval( ['[~,sheet_names] = xlsfinfo(''' N ''');'] ); 

%{
hw=he.Workbooks;
hworkbook=hw.Add;
set(he,'Visible',1);
%D=cd;
%hworkbook.SaveAs([D,'\',N],xlFormat);
%hworkbook.SaveAs(['c:','\',N],xlFormat);
%hworkbook.Close(false);
hsheets=get(hworkbook,'Sheets');
hsheet=Item(hsheets,1);
hrange=Range(hsheet,'D6');
hrange.Select;
hsheet.Paste;
%}
 warning('off','MATLAB:xlswrite:AddSheet');

 eval( ['xlswrite(''' N ''',{''original XY''},L_name,''A1'');'] );
 eval( ['xlswrite(''' N ''',L_xy,L_name,''A2:B' num2str(length(L_xy)+1) ''');'] );
 
 eval( ['xlswrite(''' N ''',{''cell wall''},L_name,''D1'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.xy(1:2,:)'',L_name,''D2:E' num2str(length(Lobe_result.xy)+1) ''');'] );
 
 eval( ['xlswrite(''' N ''',{''convex hull''},L_name,''G1'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.xy(1:2,Lobe_result.klobe(:,1)'')'',L_name,''G2:H' num2str(length(Lobe_result.klobe)+1) ''');'] );
 
 eval( ['xlswrite(''' N ''',{''most likely lobe locations''},L_name,''J1'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.xy(1:2,Lobe_result.klobe_e(:,1)'')'',L_name,''J2:K' num2str(length(Lobe_result.klobe_e)+1) ''');'] );
 
 eval( ['xlswrite(''' N ''',{''lobe number''},L_name,''M1'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.klobe_num,L_name,''O1'');'] );
 eval( ['xlswrite(''' N ''',{''compactness''},L_name,''M2'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.compactness,L_name,''O2'');'] );
 eval( ['xlswrite(''' N ''',{''roundness''},L_name,''M3'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.roundness,L_name,''O3'');'] );
 eval( ['xlswrite(''' N ''',{''convexity''},L_name,''M4'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.convexity,L_name,''O4'');'] );
 eval( ['xlswrite(''' N ''',{''solidity''},L_name,''M5'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.solidity,L_name,''O5'');'] );
 
 eval( ['xlswrite(''' N ''',{''area''},L_name,''M7'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.area,L_name,''O7'');'] );
 
 eval( ['xlswrite(''' N ''',{''convex area''},L_name,''M8'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.C_area,L_name,''O8'');'] );
 
 eval( ['xlswrite(''' N ''',{''perimeter''},L_name,''M9'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.perimeter,L_name,''O9'');'] );
 
 eval( ['xlswrite(''' N ''',{''convex perimeter''},L_name,''M10'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.C_perimeter,L_name,''O10'');'] );
 
 eval( ['xlswrite(''' N ''',{''mean_radius''},L_name,''M11'');'] );
 eval( ['xlswrite(''' N ''',Lobe_result.mean_radius,L_name,''O11'');'] );


%{
Excel = actxserver('Excel.Application');
set(Excel, 'Visible', 1);
Workbooks = Excel.Workbooks;
Workbook = invoke(Workbooks, 'Add');
Activesheet = Excel.Activesheet;

ActivesheetRange = get(Activesheet,'Range','A1:A406');
set(ActivesheetRange, 'Value', Lobe_result.x_cell');
recent_location='C:\';
eval([ 'invoke(Workbook, ''SaveAs'', ''myfile' i '.xls'');' ]);
%eval([ 'invoke(Workbook, ''SaveAs'',''' recent_location '/myfile' i '.xls'');' ]);
Workbook.Saved = 1;
invoke(Workbook, 'Close');
invoke(Excel, 'Quit');
delete(Excel);
 %}
end
function Save_excel_mac(p,L_xy,L_name,Lobe_result,excel_name)

% Add Java POI Libs to matlab javapath
javaaddpath('poi_library/poi-3.8-20120326.jar');
javaaddpath('poi_library/poi-ooxml-3.8-20120326.jar');
javaaddpath('poi_library/poi-ooxml-schemas-3.8-20120326.jar');
javaaddpath('poi_library/xmlbeans-2.3.0.jar');
javaaddpath('poi_library/dom4j-1.6.1.jar');
javaaddpath('poi_library/stax-api-1.0.1.jar');

N_name=p.excel_name;
%he=actxserver('Excel.Application');
%excelVersion = str2double(he.Version);
D=cd;
%if excelVersion < 12
%	excelExtension = '.xls';
%else
	excelExtension = '.xlsx';
%end
%{
switch excelExtension
	case '.xls' %xlExcel8 or xlWorkbookNormal
	   xlFormat = -4143;
	case '.xlsb' %xlExcel12
	   xlFormat = 50;
	case '.xlsx' %xlOpenXMLWorkbook
	   xlFormat = 51;
	case '.xlsm' %xlOpenXMLWorkbookMacroEnabled 
	   xlFormat = 52;
	otherwise
	   xlFormat = -4143;
end
%}
N=[N_name excelExtension];
%eval( ['[~,sheet_names] = xlsfinfo(''' N ''');'] ); 
%he.Workbooks.Open(fullfile(D, N));
%Sheets = he.ActiveWorkbook.Sheets;
%he.ActiveWorkbook.Worksheets.Item('AAA').Delete;
%he.ActiveWorkbook.Save;
%he.ActiveWorkbook.Close;
%he.Quit;
%he.delete;

%{
hw=he.Workbooks;
hworkbook=hw.Add;
set(he,'Visible',1);
%D=cd;
%hworkbook.SaveAs([D,'\',N],xlFormat);
%hworkbook.SaveAs(['c:','\',N],xlFormat);
%hworkbook.Close(false);
hsheets=get(hworkbook,'Sheets');
hsheet=Item(hsheets,1);
hrange=Range(hsheet,'D6');
hrange.Select;
hsheet.Paste;
%}
 warning('off','MATLAB:xlswrite:AddSheet');
 eval( ['xlwrite(''' N ''',{''original XY''},L_name,''A1'');'] );
 eval( ['xlwrite(''' N ''',L_xy,L_name,''A2:B' num2str(length(L_xy)+1) ''');'] );
 

 eval( ['xlwrite(''' N ''',{''cell wall''},L_name,''D1'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.xy(1:2,:)'',L_name,''D2:E' num2str(length(Lobe_result.xy)+1) ''');'] );
 
 eval( ['xlwrite(''' N ''',{''convex hull''},L_name,''G1'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.xy(1:2,Lobe_result.klobe(:,1)'')'',L_name,''G2:H' num2str(length(Lobe_result.klobe)+1) ''');'] );
 
 eval( ['xlwrite(''' N ''',{''most likely lobe locations''},L_name,''J1'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.xy(1:2,Lobe_result.klobe_e(:,1)'')'',L_name,''J2:K' num2str(length(Lobe_result.klobe_e)+1) ''');'] );
 
 eval( ['xlwrite(''' N ''',{''lobe number''},L_name,''M1'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.klobe_num,L_name,''O1'');'] );
 eval( ['xlwrite(''' N ''',{''compactness''},L_name,''M2'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.compactness,L_name,''O2'');'] );
 eval( ['xlwrite(''' N ''',{''roundness''},L_name,''M3'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.roundness,L_name,''O3'');'] );
 eval( ['xlwrite(''' N ''',{''convexity''},L_name,''M4'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.convexity,L_name,''O4'');'] );
 eval( ['xlwrite(''' N ''',{''solidity''},L_name,''M5'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.solidity,L_name,''O5'');'] );
 
 eval( ['xlwrite(''' N ''',{''area''},L_name,''M7'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.area,L_name,''O7'');'] );
 
 eval( ['xlwrite(''' N ''',{''convex area''},L_name,''M8'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.C_area,L_name,''O8'');'] );
 
 eval( ['xlwrite(''' N ''',{''perimeter''},L_name,''M9'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.perimeter,L_name,''O9'');'] );
 
 eval( ['xlwrite(''' N ''',{''convex perimeter''},L_name,''M10'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.C_perimeter,L_name,''O10'');'] );
 
 eval( ['xlwrite(''' N ''',{''mean_radius''},L_name,''M11'');'] );
 eval( ['xlwrite(''' N ''',Lobe_result.mean_radius,L_name,''O11'');'] );
 
%{
Excel = actxserver('Excel.Application');
set(Excel, 'Visible', 1);
Workbooks = Excel.Workbooks;
Workbook = invoke(Workbooks, 'Add');
Activesheet = Excel.Activesheet;

ActivesheetRange = get(Activesheet,'Range','A1:A406');
set(ActivesheetRange, 'Value', Lobe_result.x_cell');
recent_location='C:\';
eval([ 'invoke(Workbook, ''SaveAs'', ''myfile' i '.xls'');' ]);
%eval([ 'invoke(Workbook, ''SaveAs'',''' recent_location '/myfile' i '.xls'');' ]);
Workbook.Saved = 1;
invoke(Workbook, 'Close');
invoke(Excel, 'Quit');
delete(Excel);
 %}
end
function [excel_name,p]=excel_name_prod(p)
%he=actxserver('Excel.Application');
%excelVersion = str2double(he.Version);
%if excelVersion < 12
%	excelExtension = '.xls';
%else
	excelExtension = '.xlsx';
%end
if p.com==0
A=find(p.Data_folder=='/');
elseif p.com==1
A=find(p.Data_folder=='\');
end
%p.excel_name=p.Data_folder(1,A(1,end)+1:end);

excel_name=[p.excel_name excelExtension];
end
function excel_output(excel_name,Lob_xy)
eval( ['[~,sheet_names] = xlsfinfo(''' excel_name ''');'] );
[~,a]=size(sheet_names);[b,~]=size(Lob_xy);
for j=1:b
    lobe_list{1,j}=Lob_xy{j,2};
end

for i=1:a
    nn=strcmp (sheet_names{i}, lobe_list);
    if nn==0
        xls_delete_sheets(excel_name,sheet_names{i});
    end
end

end
function excel_output_mac(excel_name,Lob_xy)
eval( ['[~,sheet_names] = xlsfinfo(''' excel_name ''');'] );
[~,a]=size(sheet_names);[b,~]=size(Lob_xy);
for j=1:b
    lobe_list{1,j}=Lob_xy{j,2};
end

for i=1:a
    nn=strcmp (sheet_names{i}, lobe_list);
    if nn==0
        xls_delete_sheets(excel_name,sheet_names{i});
    end
end

end

function result=summary_result1()
result.averageVoting=zeros(1,18);result.modeVoting=zeros(1,18);result.klobe_num=zeros(1,18);
result.Lb_Sensitivity=zeros(1,18);result.Lb_Specificity=zeros(1,18);result.Lb_Accuracy=zeros(1,18);
result.area=zeros(1,18);result.C_area=zeros(1,18);result.perimeter=zeros(1,18);result.C_perimeter=zeros(1,18);
result.mean_radius=zeros(1,18);
result.compactness=zeros(1,18);result.roundness=zeros(1,18);result.convexity=zeros(1,18);result.solidity=zeros(1,18);


%load All_result1_7_01.mat
for i=1:18
   eval(['load Real_6_' num2str(i) '_1.mat;']);
   eval(['out' num2str(i) '=out;']);
   clear out
end
   eval(['load Real_6_' num2str(12) '_1.mat;']);
   eval(['out' num2str(4) '=out;']);
   eval(['load Real_6_' num2str(2) '_1.mat;']);
   eval(['out' num2str(6) '=out;']);
   clear out

for i=1:18
eval(['result.averageVoting(1,i)=mean(out' num2str(i) '.t1.lobe_number_voting);']);
eval(['result.modeVoting(1,i)=mode(out' num2str(i) '.t1.lobe_number_voting);']);
eval(['result.klobe_num(1,i)=out' num2str(i) '.t1.klobe_num;']);

eval(['result.Lb_Sensitivity(1,i)=out' num2str(i) '.t1.Lb_Sensitivity;']);
eval(['result.Lb_Specificity(1,i)=out' num2str(i) '.t1.Lb_Specificity;']);
eval(['result.Lb_Accuracy(1,i)=out' num2str(i) '.t1.Lb_Accuracy;']);
eval(['result.Lb_FDR(1,i)=out' num2str(i) '.t1.Lb_FDR;']);

eval(['result.area(1,i)=out' num2str(i) '.t1.area;']);
eval(['result.C_area(1,i)=out' num2str(i) '.t1.C_area;']);
eval(['result.perimeter(1,i)=out' num2str(i) '.t1.perimeter;']);
eval(['result.C_perimeter(1,i)=out' num2str(i) '.t1.C_perimeter;']);
eval(['result.mean_radius(1,i)=out' num2str(i) '.t1.mean_radius;']);

eval(['result.compactness(1,i)=out' num2str(i) '.t1.compactness;']);
eval(['result.roundness(1,i)=out' num2str(i) '.t1.roundness;']);
eval(['result.convexity(1,i)=out' num2str(i) '.t1.convexity;']);
eval(['result.solidity(1,i)=out' num2str(i) '.t1.solidity;']);
end
end
function All_Sens=summary_result3(out)
% All_Sens=summary_result3(out);

nn=100;
All_Sens=zeros(3,nn);

for i=1:100
    eval(['matrix=out.t' num2str(i) '.lb_result;']);
    %Accuracy=(matrix(1,1)+matrix(2,2))/(matrix(1,1)+matrix(2,2)+matrix(1,2)+matrix(2,1));
    eval(['All_Sens(1,' num2str(i) ')=out.t' num2str(i) '.Lb_Accuracy;']);
    eval(['All_Sens(2,' num2str(i) ')=out.t' num2str(i) '.Lb_Sensitivity;']);
    eval(['All_Sens(3,' num2str(i) ')=out.t' num2str(i) '.Lb_Specificity;']);
    eval(['All_Sens(4,' num2str(i) ')=out.t' num2str(i) '.lobe_number_voting(1,1);']);
end
end
function result=summary_dataset()

%load All_result1_7_01.mat
dataset_cell{1}=[1 2 3 4];    % dataset 1
dataset_cell{2}=[1,2,3,4,5,6,7,22,28,29,30,31,38,39,40];   % dataset 2
dataset_cell{3}=[1,17,2,26,28,29,3,37,39,44];  % dataset 3
dataset_cell{4}=[1,3,6,8,9,10,11,13,14,16,17,18,19,20];  % dataset 4

Summary.area{4,2}=[];Summary.C_area{4,2}=[];Summary.perimeter{4,2}=[];Summary.C_perimeter{4,2}=[];
Summary.mean_radius{4,2}=[];Summary.compactness{4,2}=[];Summary.roundness{4,2}=[];
Summary.convexity{4,2}=[];Summary.solidity{4,2}=[];Summary.klobe_num{4,2}=[];

for d=1:4
    ni=size(dataset_cell{d},2);
    for i=1:ni
        for j=1:3
            for tt=1:2
        eval(['load Real_' num2str(d) '_' num2str(dataset_cell{1,d}(i)) '_' num2str(j) '.mat;']);
        eval(['Summary.klobe_num{d,tt}=[Summary.klobe_num{d,tt} out.t' num2str(tt) '.klobe_num];']);    
        eval(['Summary.area{d,tt}=[Summary.area{d,tt} out.t' num2str(tt) '.area];']);
        eval(['Summary.C_area{d,tt}=[Summary.C_area{d,tt} out.t' num2str(tt) '.C_area];']);
        eval(['Summary.perimeter{d,tt}=[Summary.perimeter{d,tt} out.t' num2str(tt) '.perimeter];']);
        eval(['Summary.C_perimeter{d,tt}=[Summary.C_perimeter{d,tt} out.t' num2str(tt) '.C_perimeter];']);
        eval(['Summary.mean_radius{d,tt}=[Summary.mean_radius{d,tt} out.t' num2str(tt) '.mean_radius];']);
        eval(['Summary.compactness{d,tt}=[Summary.compactness{d,tt} out.t' num2str(tt) '.compactness];']);
        eval(['Summary.roundness{d,tt}=[Summary.roundness{d,tt} out.t' num2str(tt) '.roundness];']);
        eval(['Summary.convexity{d,tt}=[Summary.convexity{d,tt} out.t' num2str(tt) '.convexity];']);
        eval(['Summary.solidity{d,tt}=[Summary.solidity{d,tt} out.t' num2str(tt) '.solidity];']);
        clear out;
            end
        end
    end
end
clear tt ni j i d dataset_cell;
end
function box_plot_draw()
%function box_plot_draw()

% Script for draw box plot figure.

Sensitivity_18cell=[result31.Lb_Sensitivity;result32.Lb_Sensitivity;result33.Lb_Sensitivity;...
    result51.Lb_Sensitivity;result52.Lb_Sensitivity;result53.Lb_Sensitivity;...
    result71.Lb_Sensitivity;result72.Lb_Sensitivity;result73.Lb_Sensitivity];
FDR_18cell=[result31.Lb_FDR;result32.Lb_FDR;result33.Lb_FDR;...
    result51.Lb_FDR;result52.Lb_FDR;result53.Lb_FDR;...
    result71.Lb_FDR;result72.Lb_FDR;result73.Lb_FDR];

boxplot([result31.Lb_Sensitivity(1,:)',result32.Lb_Sensitivity(1,:)',result33.Lb_Sensitivity(1,:)',result51.Lb_Sensitivity(1,:)',result52.Lb_Sensitivity(1,:)'...
,result53.Lb_Sensitivity(1,:)',result71.Lb_Sensitivity(1,:)',result72.Lb_Sensitivity(1,:)',result73.Lb_Sensitivity(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)'});

boxplot([result31.Lb_FDR(1,:)',result32.Lb_FDR(1,:)',result33.Lb_FDR(1,:)',result51.Lb_FDR(1,:)',result52.Lb_FDR(1,:)'...
,result53.Lb_FDR(1,:)',result71.Lb_FDR(1,:)',result72.Lb_FDR(1,:)',result73.Lb_FDR(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)'});

boxplot([Sensitivity_18cell(:,1) Sensitivity_18cell(:,2) Sensitivity_18cell(:,3) Sensitivity_18cell(:,4) Sensitivity_18cell(:,5) Sensitivity_18cell(:,6)...
    Sensitivity_18cell(:,7) Sensitivity_18cell(:,8) Sensitivity_18cell(:,9) Sensitivity_18cell(:,10) Sensitivity_18cell(:,11) Sensitivity_18cell(:,12)...
    Sensitivity_18cell(:,13) Sensitivity_18cell(:,14) Sensitivity_18cell(:,15) Sensitivity_18cell(:,16) Sensitivity_18cell(:,17) Sensitivity_18cell(:,18)]...
    ,{'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10','#11','#12','#13','#14','#15','#16','#17','#18'});

boxplot([FDR_18cell(:,1) FDR_18cell(:,2) FDR_18cell(:,3) FDR_18cell(:,4) FDR_18cell(:,5) FDR_18cell(:,6)...
    FDR_18cell(:,7) FDR_18cell(:,8) FDR_18cell(:,9) FDR_18cell(:,10) FDR_18cell(:,11) FDR_18cell(:,12)...
    FDR_18cell(:,13) FDR_18cell(:,14) FDR_18cell(:,15) FDR_18cell(:,16) FDR_18cell(:,17) FDR_18cell(:,18)]...
    ,{'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10','#11','#12','#13','#14','#15','#16','#17','#18'});


%{
boxplot([result_x1y1.Lb_Sensitivity(1,:)',result_x1y3.Lb_Sensitivity(1,:)',result_x1y5.Lb_Sensitivity(1,:)',result_x1y7.Lb_Sensitivity(1,:)',result_x3y1.Lb_Sensitivity(1,:)'...
,result_x5y1.Lb_Sensitivity(1,:)',result_x7y1.Lb_Sensitivity(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)'});
boxplot([result31.Lb_Sensitivity(1,:)',result32.Lb_Sensitivity(1,:)',result33.Lb_Sensitivity(1,:)',result51.Lb_Sensitivity(1,:)',result52.Lb_Sensitivity(1,:)'...
,result53.Lb_Sensitivity(1,:)',result71.Lb_Sensitivity(1,:)',result72.Lb_Sensitivity(1,:)',result73.Lb_Sensitivity(1,:)'...
,result91.Lb_Sensitivity(1,:)',result92.Lb_Sensitivity(1,:)',result93.Lb_Sensitivity(1,:)']...
,{'3_1','3_2','3_3','5_1','5_2','5_3','7_1','7_2','7_3','9_1','9_2','9_3'});
    
    boxplot([result71_dif1.Lb_Sensitivity(1,:)',result71_dif2.Lb_Sensitivity(1,:)',result71_dif3.Lb_Sensitivity(1,:)',result71_dif4.Lb_Sensitivity(1,:)',result71_dif5.Lb_Sensitivity(1,:)'...
    ,result71_dif6.Lb_Sensitivity(1,:)',result71_dif7.Lb_Sensitivity(1,:)',result71_p10.Lb_Sensitivity(1,:)',result71_p15.Lb_Sensitivity(1,:)'...
    ,result71_p20.Lb_Sensitivity(1,:)',result71_p25.Lb_Sensitivity(1,:)',result71_p30.Lb_Sensitivity(1,:)',result71_p35.Lb_Sensitivity(1,:)',result71_p40.Lb_Sensitivity(1,:)']...
    ,{'dif1','dif2','dif3','dif4','dif5','dif6','dif7','p10','p15','p20','p25','p30','p35','p40'});
    
boxplot([result31.Lb_Accuracy(1,:)',result32.Lb_Accuracy(1,:)',result33.Lb_Accuracy(1,:)',result51.Lb_Accuracy(1,:)',result52.Lb_Accuracy(1,:)'...
,result53.Lb_Accuracy(1,:)',result71.Lb_Accuracy(1,:)',result72.Lb_Accuracy(1,:)',result73.Lb_Accuracy(1,:)']...
,{'3_1','3_2','3_3','5_1','5_2','5_3','7_1','7_2','7_3'});

boxplot([result31.Lb_Sensitivity(1,:)',result32.Lb_Sensitivity(1,:)',result33.Lb_Sensitivity(1,:)',result51.Lb_Sensitivity(1,:)',result52.Lb_Sensitivity(1,:)'...
    ,result53.Lb_Sensitivity(1,:)',result71.Lb_Sensitivity(1,:)',result72.Lb_Sensitivity(1,:)',result73.Lb_Sensitivity(1,:)']...
    ,{'3_1','3_2','3_3','5_1','5_2','5_3','7_1','7_2','7_3'});

boxplot([result31.Lb_Specificity(1,:)',result32.Lb_Specificity(1,:)',result33.Lb_Specificity(1,:)',result51.Lb_Specificity(1,:)',result52.Lb_Specificity(1,:)'...
    ,result53.Lb_Specificity(1,:)',result71.Lb_Specificity(1,:)',result72.Lb_Specificity(1,:)',result73.Lb_Specificity(1,:)']...
    ,{'3_1','3_2','3_3','5_1','5_2','5_3','7_1','7_2','7_3'});
%}
%{
Accuracy_18cell=[result31.Lb_Accuracy;result32.Lb_Accuracy;result33.Lb_Accuracy;...
    result51.Lb_Accuracy;result52.Lb_Accuracy;result53.Lb_Accuracy;...
    result71.Lb_Accuracy;result72.Lb_Accuracy;result73.Lb_Accuracy];
Sensitivity_18cell=[result31.Lb_Sensitivity;result32.Lb_Sensitivity;result33.Lb_Sensitivity;...
    result51.Lb_Sensitivity;result52.Lb_Sensitivity;result53.Lb_Sensitivity;...
    result71.Lb_Sensitivity;result72.Lb_Sensitivity;result73.Lb_Sensitivity];
Specificity_18cell=[result31.Lb_Specificity;result32.Lb_Specificity;result33.Lb_Specificity;...
    result51.Lb_Specificity;result52.Lb_Specificity;result53.Lb_Specificity;...
    result71.Lb_Specificity;result72.Lb_Specificity;result73.Lb_Specificity];
%}
%{
boxplot([Accuracy_18cell(:,1) Accuracy_18cell(:,2) Accuracy_18cell(:,3) Accuracy_18cell(:,4) Accuracy_18cell(:,5) Accuracy_18cell(:,6)...
    Accuracy_18cell(:,7) Accuracy_18cell(:,8) Accuracy_18cell(:,9) Accuracy_18cell(:,10) Accuracy_18cell(:,11) Accuracy_18cell(:,12)...
    Accuracy_18cell(:,13) Accuracy_18cell(:,14) Accuracy_18cell(:,15) Accuracy_18cell(:,16) Accuracy_18cell(:,17) Accuracy_18cell(:,18)]...
    ,{'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10','#11','#12','#13','#14','#15','#16','#17','#18'});
%}
%{
boxplot([Sensitivity_18cell(:,1) Sensitivity_18cell(:,2) Sensitivity_18cell(:,3) Sensitivity_18cell(:,4) Sensitivity_18cell(:,5) Sensitivity_18cell(:,6)...
    Sensitivity_18cell(:,7) Sensitivity_18cell(:,8) Sensitivity_18cell(:,9) Sensitivity_18cell(:,10) Sensitivity_18cell(:,11) Sensitivity_18cell(:,12)...
    Sensitivity_18cell(:,13) Sensitivity_18cell(:,14) Sensitivity_18cell(:,15) Sensitivity_18cell(:,16) Sensitivity_18cell(:,17) Sensitivity_18cell(:,18)]...
    ,{'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10','#11','#12','#13','#14','#15','#16','#17','#18'});
%}
%{
boxplot([Specificity_18cell(:,1) Specificity_18cell(:,2) Specificity_18cell(:,3) Specificity_18cell(:,4) Specificity_18cell(:,5) Specificity_18cell(:,6)...
    Specificity_18cell(:,7) Specificity_18cell(:,8) Specificity_18cell(:,9) Specificity_18cell(:,10) Specificity_18cell(:,11) Specificity_18cell(:,12)...
    Specificity_18cell(:,13) Specificity_18cell(:,14) Specificity_18cell(:,15) Specificity_18cell(:,16) Specificity_18cell(:,17) Specificity_18cell(:,18)]...
    ,{'#1','#2','#3','#4','#5','#6','#7','#8','#9','#10','#11','#12','#13','#14','#15','#16','#17','#18'});
%}
%{
boxplot([result31.Lb_Accuracy(1,:)',result32.Lb_Accuracy(1,:)',result33.Lb_Accuracy(1,:)',result51.Lb_Accuracy(1,:)',result52.Lb_Accuracy(1,:)'...
,result53.Lb_Accuracy(1,:)',result71.Lb_Accuracy(1,:)',result72.Lb_Accuracy(1,:)',result73.Lb_Accuracy(1,:)'...
,result31_x1y3.Lb_Accuracy(1,:)',result31_x1y5.Lb_Accuracy(1,:)',result31_x3y1.Lb_Accuracy(1,:)',result31_x5y1.Lb_Accuracy(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)','x1y3','x1y5','x3y1','x5y1'});

boxplot([result31.Lb_Sensitivity(1,:)',result32.Lb_Sensitivity(1,:)',result33.Lb_Sensitivity(1,:)',result51.Lb_Sensitivity(1,:)',result52.Lb_Sensitivity(1,:)'...
,result53.Lb_Sensitivity(1,:)',result71.Lb_Sensitivity(1,:)',result72.Lb_Sensitivity(1,:)',result73.Lb_Sensitivity(1,:)'...
,result31_x1y3.Lb_Sensitivity(1,:)',result31_x1y5.Lb_Sensitivity(1,:)',result31_x3y1.Lb_Sensitivity(1,:)',result31_x5y1.Lb_Sensitivity(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)','x1y3','x1y5','x3y1','x5y1'});

boxplot([result31.Lb_Specificity(1,:)',result32.Lb_Specificity(1,:)',result33.Lb_Specificity(1,:)',result51.Lb_Specificity(1,:)',result52.Lb_Specificity(1,:)'...
,result53.Lb_Specificity(1,:)',result71.Lb_Specificity(1,:)',result72.Lb_Specificity(1,:)',result73.Lb_Specificity(1,:)'...
,result31_x1y3.Lb_Specificity(1,:)',result31_x1y5.Lb_Specificity(1,:)',result31_x3y1.Lb_Specificity(1,:)',result31_x5y1.Lb_Specificity(1,:)']...
,{'(3,1)','(3,2)','(3,3)','(5,1)','(5,2)','(5,3)','(7,1)','(7,2)','(7,3)','x1y3','x1y5','x3y1','x5y1'});
%}
%{
boxplot([All_Sens_10_10(1,:)',All_Sens_10_10_x1y3(1,:)',All_Sens_10_10_x1y5(1,:)',All_Sens_10_10_x3y1(1,:)',All_Sens_10_10_x5y1(1,:)'...
    ,All_Sens_13_10(1,:)' ,All_Sens_13_10_x1y3(1,:)' ,All_Sens_13_10_x1y5(1,:)' ,All_Sens_13_10_x3y1(1,:)' ,All_Sens_13_10_x5y1(1,:)'...
    ,All_Sens_10_10(1,:)' ,All_Sens_16_10_x1y3(1,:)' ,All_Sens_16_10_x1y5(1,:)' ,All_Sens_16_10_x3y1(1,:)' ,All_Sens_16_10_x5y1(1,:)']...
    ,{'10','10(x1y3)','10(x1y5)','10(x3y1)','10(x5y1)','13','13(x1y3)','13(x1y5)','13(x3y1)','13(x5y1)','16','16(x1y3)','16(x1y5)','16(x3y1)','16(x5y1)'});
%}
%{
boxplot([All_Sens_10_10(3,:)',All_Sens_10_10_x1y3(3,:)',All_Sens_10_10_x1y5(3,:)',All_Sens_10_10_x3y1(3,:)',All_Sens_10_10_x5y1(3,:)'...
    ,All_Sens_13_10(3,:)' ,All_Sens_13_10_x1y3(3,:)' ,All_Sens_13_10_x1y5(3,:)' ,All_Sens_13_10_x3y1(3,:)' ,All_Sens_13_10_x5y1(3,:)'...
    ,All_Sens_10_10(3,:)' ,All_Sens_16_10_x1y3(3,:)' ,All_Sens_16_10_x1y5(3,:)' ,All_Sens_16_10_x3y1(3,:)' ,All_Sens_16_10_x5y1(3,:)']...
    ,{'10','10(x1y3)','10(x1y5)','10(x3y1)','10(x5y1)','13','13(x1y3)','13(x1y5)','13(x3y1)','13(x5y1)','16','16(x1y3)','16(x1y5)','16(x3y1)','16(x5y1)'});
%}

%{
boxplot([Summary.compactness{1,1}',Summary.compactness{1,2}',Summary.compactness{2,1}',Summary.compactness{2,2}',Summary.compactness{3,1}'...
    ,Summary.compactness{3,2}' ,Summary.compactness{4,1}' ,Summary.compactness{4,2}']...
    ,{'D3','D5','D2','D5','h38','h55','h38','h55'});
%}
%{
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.compactness{3,1}';Summary.compactness{4,1}';Summary.compactness{3,2}';Summary.compactness{4,2}';Summary.compactness{2,1}'...
    ;Summary.compactness{1,1}' ;Summary.compactness{1,2}' ;Summary.compactness{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.roundness{3,1}';Summary.roundness{4,1}';Summary.roundness{3,2}';Summary.roundness{4,2}';Summary.roundness{2,1}'...
    ;Summary.roundness{1,1}' ;Summary.roundness{1,2}' ;Summary.roundness{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.convexity{3,1}';Summary.convexity{4,1}';Summary.convexity{3,2}';Summary.convexity{4,2}';Summary.convexity{2,1}'...
    ;Summary.convexity{1,1}' ;Summary.convexity{1,2}' ;Summary.convexity{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.solidity{3,1}';Summary.solidity{4,1}';Summary.solidity{3,2}';Summary.solidity{4,2}';Summary.solidity{2,1}'...
    ;Summary.solidity{1,1}' ;Summary.solidity{1,2}' ;Summary.solidity{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.klobe_num{3,1}';Summary.klobe_num{4,1}';Summary.klobe_num{3,2}';Summary.klobe_num{4,2}';Summary.klobe_num{2,1}'...
    ;Summary.klobe_num{1,1}' ;Summary.klobe_num{1,2}' ;Summary.klobe_num{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.area{3,1}';Summary.area{4,1}';Summary.area{3,2}';Summary.area{4,2}';Summary.area{2,1}'...
    ;Summary.area{1,1}' ;Summary.area{1,2}' ;Summary.area{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.C_area{3,1}';Summary.C_area{4,1}';Summary.C_area{3,2}';Summary.C_area{4,2}';Summary.C_area{2,1}'...
    ;Summary.C_area{1,1}' ;Summary.C_area{1,2}' ;Summary.C_area{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.perimeter{3,1}';Summary.perimeter{4,1}';Summary.perimeter{3,2}';Summary.perimeter{4,2}';Summary.perimeter{2,1}'...
    ;Summary.perimeter{1,1}' ;Summary.perimeter{1,2}' ;Summary.perimeter{2,2}']...
    ,group);
group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.mean_radius{3,1}';Summary.mean_radius{4,1}';Summary.mean_radius{3,2}';Summary.mean_radius{4,2}';Summary.mean_radius{2,1}'...
    ;Summary.mean_radius{1,1}' ;Summary.mean_radius{1,2}' ;Summary.mean_radius{2,2}']...
    ,group);

group = [repmat({'h38'}, 10, 1); repmat({'h38'}, 14, 1); repmat({'h55'}, 10, 1); repmat({'h55'}, 14, 1); repmat({'D2'}, 15, 1); repmat({'D3'}, 4, 1); repmat({'D5'}, 4, 1); repmat({'D5'}, 15, 1)];
boxplot([Summary.convexity{3,1}';Summary.convexity{4,1}';Summary.convexity{3,2}';Summary.convexity{4,2}';Summary.convexity{2,1}'...
    ;Summary.convexity{1,1}' ;Summary.convexity{1,2}' ;Summary.convexity{2,2}']...
    ,group);
%}

end
function B=get_angle(kkk,out)
% [xy, A]=get_angle(kkk,out)
xy=zeros(1,2);
xx=out.t1.xy_o(1,kkk);yy=out.t1.xy_o(2,kkk);
    if xx>=0&&yy>=0
        A=atan(abs(yy)/abs(xx));
        xy=[xx yy];
    elseif xx<0&&yy>=0
        A=pi-atan(abs(yy)/abs(xx));
        xy=[xx yy];
    elseif xx<0&&yy<0
        A=pi+atan(abs(yy)/abs(xx));
        xy=[xx yy];
    elseif  xx>=0&&yy<0
        A=2*pi-atan(abs(yy)/abs(xx));
        xy=[xx yy];
    end
    B=[A xy];
end
function [t1_ms,t2_ms,t1dt2_meanstd]=t1dt2_population_meanstd(input,option)
% [t1_ms,t2_ms,t1dt2_meanstd]=t1dt2_population_meanstd(input,1);

[a,~]=size(input);
t1dt2_meanstd=zeros(a,2);t1_ms=zeros(a,2);t2_ms=zeros(a,2);
for i=1:a
    if option==1
        t1t2_ratio=input{i,2}./input{i,1};
    elseif option==2
        t1t2_ratio=input{i,2}-input{i,1};
    end
    t1_ms(i,1)=mean(input{i,1});t1_ms(i,2)=std(input{i,1});
    t2_ms(i,1)=mean(input{i,2});t2_ms(i,2)=std(input{i,2});
    t1dt2_meanstd(i,1)=mean(t1t2_ratio);t1dt2_meanstd(i,2)=std(t1t2_ratio);
end
end

% === Important function ===
function [GlobalcontP,Gp] = findContP(contP,x_cell,y_cell,num)
% make the control points on segment or boundary.
    [n,m]=size(contP);GlobalcontP=zeros(n,4);Gp=zeros(1,num);
    for i=1:n
        D=(x_cell-contP(i,1)).^2+(y_cell-contP(i,2)).^2;
        [dd,GlobalcontP(i,1)]=min(D);GlobalcontP(i,3)=round(GlobalcontP(i,1)*num/length(x_cell));
        E=(x_cell-contP(i,3)).^2+(y_cell-contP(i,4)).^2;
        [dd,GlobalcontP(i,2)]=min(E);GlobalcontP(i,4)=round(GlobalcontP(i,2)*num/length(x_cell));
        Gp(1,GlobalcontP(i,3):GlobalcontP(i,4))=1;
    end
end
function out=com_dist(in,n)
     %  enlarge distance matrix to n size
    out=zeros(1, n);
    [a,b]=size(in);
    for i=1:n
        temp=round(i/n*b);
        if temp<1
             out(1,i)=in(1,1);
        elseif temp>b
            out(1,i)=in(1,b);
        else
            out(1,i)=in(1,temp);
        end
    end
end 
function [out1,center]=movC(input1,center)
% move the center of cell into (0,0)

% make sure input1 is 2xn
if size(input1,1)>3
    input1=input1';
else
end
% connect head with tel
if input1(:,1)==input1(:,end)
else
    input1= [input1 input1(:,1)];
end
[a,b]=size(input1);

% calculate weighted center position
if center(1,1)==0&&center(2,1)==0
    center=mean(input1(1:2,:),2);   
end
out1=input1;

% move xy coordinates into new xy(center=(0,0))
out1(1:2,:)=input1(1:2,:)-[ones(1,b)*center(1,1);ones(1,b)*center(2,1)];
end
function [out3,ratio_r,center,first_point]=movC_resize(input1,center,p)
% move the center of cell into (0,0) and resize as mean radius = radius

% make sure input1 is 2xn
if size(input1,1)>3 
    input1=input1';
end
radius=p.radius;

% make sure the first point of data is lobe point
k_original=convhull(input1(1,:),input1(2,:));
first_point=1;
if k_original(1,1)~=1

    input2=[input1(:,k_original(1,1):end) input1(:,1:k_original(1,1)-1)];
    first_point=k_original(1,1);
else
    input2=input1;
end
input3=input2(:,1);
for i=1:size(input2,2)-1
    if input2(1,i)-input2(1,i+1)==0&&input2(2,i)-input2(2,i+1)==0
    else
        input3=[input3 input2(:,i+1)];
    end
end
input1=input3;
% calculate weighted center position
[a,b]=size(input1);out2=ones(a,b);
if center(1,1)==0&&center(2,1)==0
    center=mean(input1(1:2,:),2);
end
out1=input1;

% adjust p.cen when r.x or r.y change
if any(strcmp('cen',fieldnames(p)))==1
    if p.expan_x~=1||p.expan_x~=1
    pp=size(p.cen,1);
        for ppi=1:pp
            for ppj=1:size(p.cen{ppi,1},2)
                if 0<=p.cen{ppi,1}(1,ppj)&&p.cen{ppi,1}(1,ppj)<0.5*pi
                    xx=abs(p.expan_x);yy=abs(tan(p.cen{ppi,1}(1,ppj))*p.expan_y);
                elseif 0.5*pi<=p.cen{ppi,1}(1,ppj)&&p.cen{ppi,1}(1,ppj)<pi
                    xx=-abs(p.expan_x);yy=abs(tan(p.cen{ppi,1}(1,ppj))*p.expan_y);
                elseif pi<=p.cen{ppi,1}(1,ppj)&&p.cen{ppi,1}(1,ppj)<1.5*pi
                    xx=-abs(p.expan_x);yy=-abs(tan(p.cen{ppi,1}(1,ppj))*p.expan_y);
                elseif 1.5*pi<=p.cen{ppi,1}(1,ppj)&&p.cen{ppi,1}(1,ppj)<2*pi
                    xx=abs(p.expan_x);yy=-abs(tan(p.cen{ppi,1}(1,ppj))*p.expan_y);
                end                   
                p.cen{ppi,1}(1,ppj)=xytotheta(xx,yy);            
            end
        end
    end
end

% move xy coordinates into new xy(center=(0,0))
out1(1:2,:)=input1(1:2,:)-[ones(1,b)*center(1,1);ones(1,b)*center(2,1)];

%morph transform xy and y-inverse
out2=[out1(1,:)*p.expan_x;-out1(2,:)*p.expan_y];

% Calculate ratio_r
rsum=0;
for i=1:b
    rsum=rsum+sqrt(out2(1,i)^2+out2(2,i)^2);
end
ratio_r=radius/(rsum/b);

% resize as mean radius = radius
out3=out2.*ratio_r;
%ratio_xr=out3(1,1)/out1(1,1);
%ratio_yr=out3(2,1)/out1(2,1);
end
function out=segment_information(out,num,cells)
% get the distances of every segment data from data.mat(cells), and resize.  
seg_num=1000;
sn1=size(cells{1,num}.segment_hulls{1,1},2);
sn2=size(cells{1,num}.segment_hulls{1,2},2);
out.seg1=zeros(sn1,seg_num);out.seg2=zeros(sn2,seg_num);
for s1=1:sn1
    out.seg1(s1,:)=com_dist(cells{1,num}.segment_hulls{1,1}{1,s1}.distances,seg_num);
end
for s2=1:sn2
    out.seg2(s2,:)=com_dist(cells{1,num}.segment_hulls{1,2}{1,s2}.distances,seg_num);
end
end
function out=grow_s(input,k)
% expansion from center model
if k==0
     out=input;
else
      out=(1+k)*input;
end
end
function out=grow_sb(input,k)
% expansion from boundary model
avg_r=sum(sqrt((input(1,:).^2+input(2,:).^2)))/length(input(1,:));
rr=avg_r./sqrt((input(1,:).^2+input(2,:).^2));
if k==0
     out=input;
else
      out=[(rr*k+1).*input(1,:);(rr*k+1).*input(2,:)];
end
end
function out_point=grow_sb_process(input,k,inputback,center)
   kk=0.1;
   a1=(input(:,1)-input(:,2))/distance(input(:,1)',input(:,2)')*kk+input(:,2);
   a3=(input(:,3)-input(:,2))/distance(input(:,3)',input(:,2)')*kk+input(:,2);
   if distance((a1'+a3')/2,input(:,2)')<=0.5*kk
       b=[a3(2)-a1(2);a1(1)-a3(1)];
       out_point1=b/sqrt(b(1)^2+b(2)^2)*k+input(:,2);
       out_point2=-b/sqrt(b(1)^2+b(2)^2)*k+input(:,2);
   else
       out_point1=((a1+a3)/2-input(:,2))/distance((a1'+a3')/2,input(:,2)')*k+input(:,2);
       out_point2=-((a1+a3)/2-input(:,2))/distance((a1'+a3')/2,input(:,2)')*k+input(:,2);
   end
   if center==[1000;1000]
        if distance(out_point1',inputback')<distance(out_point2',inputback')
           out_point=out_point1;
       elseif distance(out_point2',inputback')<distance(out_point1',inputback')
           out_point=out_point2;
        else
           out_point=[out_point1 out_point2];
        end
  else
       if distance(out_point1',center')>distance(out_point2',center')
           out_point=out_point1;
       elseif distance(out_point2',center')>distance(out_point1',center')
           out_point=out_point2;
       else
           out_point=[out_point1 out_point2];
       end
   end
end
function out=short_move(in,k,gate)
% cut the additonal after expansion model. 
% k: search range
% gate: if distance with neighbor<gate, then delete this point
l=size(in,2);
delete_list=zeros(1,l);
for i=1:l-k+1
    D=pdist2(in(:,i+2:i+k-1)',in(:,i)');
    [a,b]=min(D);
    if a<gate
        delete_list(i:i+1+b)=1;
    end
end
out=zeros(2,1);
for j=1:l
    if delete_list(j)~=1;
        out=[out in(:,j)];
    end
end
out=out(:,2:end);
end
function [k22,k4]=contrP(k1,points,gate)
    L=length(k1);
    k3=[k1(1:end-1);k1(1)+k1(end-1);k1(2)+k1(end-1)];
    k4=zeros(1,2);M=1;k2=zeros(1,4);
    for i=2:L
        if abs(k3(i)-k3(i-1))>gate||abs(k3(i+1)-k3(i))>gate
            if abs(k3(i-1)-k3(i))>gate
                k4(M,2)=k1(i);k2(M,3)=points(k1(i),1);k2(M,4)=-points(k1(i),2);
            end
            if abs(k3(i)-k3(i+1))>gate
                M=M+1;
                k4(M,1)=k1(i);k2(M,1)=points(k1(i),1);k2(M,2)=-points(k1(i),2);
            end
        end                
    end
    if k4(1,2)~=0
        k4(end,2)=k4(1,2);k2(end,3)=points(k4(end,2),1);k2(end,4)=-points(k4(end,2),2);
        k4=k4(2:end,:);k2=k2(2:end,:);
    else
        k4=k4(2:end,:);k2=k2(2:end,:);
    end
    k22=zeros(size(k2,1)*2,3);
    for i=1:size(k2,1)
        k22(2*i-1:2*i,1)=i;k22(2*i-1,2:3)=k2(i,1:2);k22(2*i,2:3)=k2(i,3:4);
    end
 end
function out=grow_noisy(input,Contp,noise_p,noise_height)
if input(:,1)==input(:,end)
    clos_check=1;
else
    clos_check=0;
end
if (noise_p(1)==0&&noise_p(2)==0)||(noise_height(1)==0&&noise_height(2)==0)
    out=input;
else
% change shape to control points
[a,b]=size(Contp);
for i=1:a
    for j=1:b
        if rand(1)>0.3
        else
        lopp=noise_p(1)+(noise_p(2)-noise_p(1))*randn(1);
        loph=noise_height(1)+(noise_height(2)-noise_height(1))*randn(1);
        input=grow_loop(input,Contp(i,j),lopp,loph);
        end
    end
end
out=input;
end
if clos_check==1;
    out(:,end)=out(:,1);
end
end
function [out,lop_range]=grow_loop(input,lop_xy,lop_p,lop_height)
% add artificial loop into boundary
if input(:,1)==input(:,end)
    clos_check=1;
else
    clos_check=0;
end
if lop_height==0||lop_p==0||lop_xy<0
    out=input;
    lop_range=[0,0];
else
    n=size(input,2);nois=ones(1,n);lop_mark=zeros(1,n);
    for i=1:n
        nois(1,i)=lop_height*gauss_distribution(i, lop_xy, lop_p)+1;
        lop_mark(1,i)=gauss_distribution(i, lop_xy, lop_p);
    end
    mm=max(lop_mark);
    for i=1:n
        if lop_mark(1,i)>0.05*mm
            lop_mark(1,i)=1;
        else
            lop_mark(1,i)=0;
        end
    end
    lop_range=[find(lop_mark==1, 1 ) find(lop_mark==1, 1, 'last' )];
    out=[nois;nois].*input;
end
if clos_check==1;
    out(:,end)=out(:,1);
end
end
function f = gauss_distribution(x, mu, s)
% create gaussian distribution
    p1 = -.5 * ((x - mu)/s) .^ 2;
    p2 = (s * sqrt(2*pi));
    f = exp(p1) ./ p2; 
end
function [curva,bend_energy, slope]=curvature(x_segment,y_segment,cL,bL)
%ex:   [curva,bend_energy]=curvature(top1(:,1),top1(:,2),5);
% cL: k in curvature; bL: L in bending energy

    n=length(x_segment);curva=zeros(1,n-2*cL);bend_energy=zeros(1,n-2*cL-bL+1);slope=zeros(1,n-cL);
    for i=1+cL:n-cL
        curva(1,i-cL)=rem(atan((y_segment(i+cL)-y_segment(i))/(x_segment(i+cL)-x_segment(i)))-atan((y_segment(i)-y_segment(i-cL))/(x_segment(i)-x_segment(i-cL))),2*pi);
    end
    for j=1:n-2*cL-bL+1
        bend_energy(1,j)=sum(curva(1,j:j+bL-1).^2)/bL;
    end
    for i=1+cL:n
        slope(1,i-cL)=rem(atan((y_segment(i)-y_segment(i-cL))/(x_segment(i)-x_segment(i-cL))),2*pi);
    end
end
function lop=createloop(lop)
if lop.lop_r<0
    lop.lop_xy=-1;
    lop.lop_p=lop.lop_p_max;
    lop.lop_height=lop.lop_height_max;
elseif lop.lop_r==0 %designed loop
    lop.lop_p=lop.lop_p_max;
    lop.lop_height=lop.lop_height_max;
    lop.lop_xy=lop.lop_xy_max;
elseif lop.lop_r==1 %random loop
    [a,b]=size(lop.contP);
    a1=randi(a);
    lop.lop_xy=randi([lop.contP(a1,1),lop.contP(a1,2)]);
    lop.lop_height=lop.lop_height_max;
    %lop.lop_height=lop.lop_height_max*rand(1,1);
    lop.lop_p=lop.lop_p_max;
    %lop.lop_p=lop.lop_p_max*rand(1,1)*0.5+lop.lop_p_max*0.5;
end
end
function k3=innercontrP(k4,points)
        c=size(k4,1);p=size(points,1);
        k3=zeros(c,3);
        for i=1:c
           dmax=0;
           for j=k4(i,1)+1:k4(i,2)-1
               d = abs(det([points(k4(i,2),:)-points(k4(i,1),:);points(j,:)-points(k4(i,1),:)])) /norm(points(k4(i,1),:)-points(k4(i,2),:));
               if d>dmax
                   dmax=d;k3(i,1)=j;
               end
           end
           if  k3(i,1)==0
           else
            k3(i,2)=points(k3(i,1),1);k3(i,3)=points(k3(i,1),2);
           end
        end
        
end
function [maxtab, mintab]=peakdet(v, delta, x)
%PEAKDET Detect peaks in a vector
%        [MAXTAB, MINTAB] = PEAKDET(V, DELTA) finds the local
%        maxima and minima ("peaks") in the vector V.
%        MAXTAB and MINTAB consists of two columns. Column 1
%        contains indices in V, and column 2 the found values.
%      
%        With [MAXTAB, MINTAB] = PEAKDET(V, DELTA, X) the indices
%        in MAXTAB and MINTAB are replaced with the corresponding
%        X-values.
%
%        A point is considered a maximum peak if it has the maximal
%        value, and was preceded (to the left) by a value lower by
%        DELTA.

% Eli Billauer, 3.4.05 (Explicitly not copyrighted).
% This function is released to the public domain; Any use is allowed.

maxtab = [];
mintab = [];

v = v(:); % Just in case this wasn't a proper vector

if nargin < 3
  x = (1:length(v))';
else 
  x = x(:);
  if length(v)~= length(x)
    error('Input vectors v and x must have same length');
  end
end
  
if (length(delta(:)))>1
  error('Input argument DELTA must be a scalar');
end

if delta <= 0
  error('Input argument DELTA must be positive');
end

mn = Inf; mx = -Inf;
mnpos = NaN; mxpos = NaN;

lookformax = 1;

for i=1:length(v)
  this = v(i);
  if this > mx, mx = this; mxpos = x(i); end
  if this < mn, mn = this; mnpos = x(i); end
  
  if lookformax
    if this < mx-delta
      maxtab = [maxtab ; mxpos mx];
      mn = this; mnpos = x(i);
      lookformax = 0;
    end  
  else
    if this > mn+delta
      mintab = [mintab ; mnpos mn];
      mx = this; mxpos = x(i);
      lookformax = 1;
    end
  end
end
end
function [series2]=trapezium_transform(series1,c1,cn)
% transform one series1 into new series2 by control points c1,cn 
%1. check matrix axis
if size(series1,1)~=2
    series1=series1';
end
if size(c1,1)~=2
    c1=c1';
end
if size(cn,1)~=2
    cn=cn';
end
L=size(series1,2);series2=zeros(2,L);
%2. start and end properies
e1_0=distance(0,0,c1(1,1),c1(2,1))/distance(0,0,series1(1,1),series1(2,1));
en_0=distance(0,0,cn(1,1),cn(2,1))/distance(0,0,series1(1,L),series1(2,L));
ddd=distance(0,0,series1(1,:),series1(2,:));
ee1=e1_0/ddd(1,1);een=en_0/ddd(1,L);
e=[ee1:(een-ee1)/(L-1):een].*ddd;

[theta1,s1]=antitan(series1(:,1),c1);[thetan,sn]=antitan(series1(:,L),cn);
if s1==1&&sn==4
    thetan=thetan-2*pi;
end
if s1==4&&sn==1
    theta1=theta1-2*pi;
end
d1=distance(c1(1,1),c1(2,1),series1(1,1),series1(2,1));
dn=distance(cn(1,1),cn(2,1),series1(1,L),series1(2,L));

for i=1:L
    series2(:,i)=tan_dist(series1(:,i),(dn-d1)/(L-1)*(i-1)+d1,(thetan-theta1)/(L-1)*(i-1)+theta1);
    %series2(:,i)=tan_dist(series1(:,i),(e(i)-1)*distance(0,0,series1(1,i),series1(2,i)),(thetan-theta1)/(L-1)*(i-1)+theta1);
end
end
function [theta,spacec0]=antitan(c0,c1)
%calculate tan angle from point c0 to c1
if size(c0,1)~=2
    c0=c0';
end
if size(c1,1)~=2
    c1=c1';
end
if c1(1,1)-c0(1,1)>=0&&c1(2,1)-c0(2,1)>=0
    spacec0=1;
elseif c1(1,1)-c0(1,1)<0&&c1(2,1)-c0(2,1)>=0
    spacec0=2;
elseif c1(1,1)-c0(1,1)<0&&c1(2,1)-c0(2,1)<0
    spacec0=3;
elseif c1(1,1)-c0(1,1)>=0&&c1(2,1)-c0(2,1)<0
    spacec0=4;
end
if spacec0==1
    theta=atan((c1(2,1)-c0(2,1))/(c1(1,1)-c0(1,1)));
elseif spacec0==2
    theta=atan((c1(2,1)-c0(2,1))/(c1(1,1)-c0(1,1)))+pi;
elseif spacec0==3
    theta=atan((c1(2,1)-c0(2,1))/(c1(1,1)-c0(1,1)))+pi;
elseif spacec0==4
    theta=atan((c1(2,1)-c0(2,1))/(c1(1,1)-c0(1,1)))+2*pi;
end
end
function c1=tan_dist(c0,d,theta)
%calculate c1 coordinate from c0 by giving distance and theta
    if size(c0,1)~=2
       c0=c0'; 
    end
    c1=zeros(2,1);
    c1(1,1)=d*cos(theta)+c0(1,1);
    c1(2,1)=d*sin(theta)+c0(2,1);
end
function segment=getsegment(segment,s6_hull,x, y, se,ztext)
[k_I k_II] = splitHull(x, y);%kk_I=[min([k_I k_II]),max([k_I k_II])];
%eval(['segment.side_' num2str(se) '_' ztext '=distanceFromHull3(x, y, s6_hull,kk_I,'''');']);
eval(['segment.side_' num2str(se) '_' ztext '=distanceFromHull3(x, y, s6_hull,k_I,'''');']);
%eval(['segment.convex_length(1,se)=segment.side_' num2str(se) '_' ztext '.hull_length;']);
%eval(['segment.convex_length(3,se)=segment.side_' num2str(se) '_' ztext '.arc_length;']);
%eval(['segment.convex_length(4,se)=segment.side_' num2str(se) '_' ztext '.max_dist;']);
ddd=0;eval(['ddd=size(segment.side_' num2str(se) '_' ztext '.distances,2);']);
if rem(ddd,2)==0
    eval(['segment.side_' num2str(se) '_' ztext '.distances_x=-0.5*ddd:0.5*ddd-1;']);
elseif rem(ddd,2)==1
    eval(['segment.side_' num2str(se) '_' ztext '.distances_x=-0.5*ddd:0.5*ddd-1;']);
end   
end
function [k_I k_II] =  splitHull(x, y)
    % Find the control points for the convex hull and split it into the hulls
    % for each side.

    % find convex hull
    k = convhull(x, y);k=convhull_redirect(k);
    % split the hull into two sides
    k_I = [];
    k_II = [];
    k_last = -inf; %k(1);
    for k_i = k'
        if k_i > k_last
            k_I(end + 1) = k_i; 
        elseif k_i < k_last
            k_II(end + 1) = k_i;
        end
        k_last = k_i;
    end
    k_II = [k_I(end) k_II];
end
function data = distanceFromHull3(x, y, s6_hull,k, adjust_hull)
        data = struct;
        direction = 2*(k(2) > k(1))-1; % needed for the iterative part below
        k = sort(k);

        data.arc = [];
        data.distances = [];
        data.x_segment = [];
        data.y_segment = [];
        data.x_hull = [];
        data.y_hull = [];
        d = 0.1;
        % d is the density of points sampled; 0.1 works well but may appear jagged

        % break the hull into bite-size pieces for analysis
        for f = 1:length(k)-1
            if isempty(data.arc) % arc_last tracks the last position along the hull so that lengths are additive
                arc_last = 0;
            else
                arc_last = data.arc(end);
            end

            k_1 = k(f);
            k_2 = k(f + 1);

            % create dense arrays of points on the hull
            hull_x = [];
            hull_y = [];
            hull_pos = [];
            distance = sqrt((x(k_1) - x(k_2)).^2 + (y(k_1) - y(k_2)).^2);
            for i = 1:size(s6_hull,2)%linspace(1, 2, ceil(distance/d))
                %[hull_x(end + 1), hull_y(end + 1)] = interp_xy(x(sort([k_1 k_2])), y(sort([k_1 k_2])), i);
               hull_x(end + 1) =s6_hull(1,i);hull_y(end + 1) =s6_hull(2,i);
                if isempty(hull_pos)
                    hull_pos(end + 1) = 0;
                else
                    hull_pos(end + 1) = hull_pos(end) + sqrt((hull_x(end) - hull_x(end - 1)).^2 + (hull_y(end) - hull_y(end - 1)).^2 );
                end
            end

            % create dense arrays of points on the segment
            wall_x = [];
            wall_y = [];
            for i = min(k_1, k_2):max(k_1, k_2)-1
                p_1 = i;
                p_2 = i + 1;
                distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                for j = linspace(p_1, p_2, ceil(distance/d))
                    [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                end
            end

            % find the closest point from a point on the hull to the segment
            for i = 1:length(wall_x)
                wx = wall_x(i);
                wy = wall_y(i);
                dist = inf;
                for j = 1:length(hull_x)
                    hx = hull_x(j);
                    hy = hull_y(j);
                    di = sqrt((wx - hx).^2 + (wy - hy).^2);
                    if di < dist
                        dist = di;
                        best = j;
                    end
                end
                data.distances(end + 1) = dist; % record distance between points
                data.arc(end + 1) = arc_last + hull_pos(best); % record distance along arc
                data.x_segment(end + 1) = wx;
                data.y_segment(end + 1) = wy;
                data.x_hull(end + 1) = hull_x(best);
                data.y_hull(end + 1) = hull_y(best);
            end
        end

        [data.arc I] = sort(data.arc);
        data.distances = data.distances(I);
        data.x_segment = data.x_segment(I);
        data.y_segment = data.y_segment(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);
        
        [data.arc I] = remove_duplicates(data.arc, data.distances);
        data.distances = data.distances(I);
        data.x_segment = data.x_segment(I);
        data.y_segment = data.y_segment(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);

        if adjust_hull

            % Search for local minima (disregarding values < the sampling distance, d).
            % If any are found, clean out arc and distances, record the x/y coordinate,
            % insert a new control point for the "hull" and recalculate the
            % distance-from-hull for either side.  Then recombine the results.

            local_min = -1;
            value = inf;

            for i = 2:length(data.arc)-1
                if data.distances(i) > d && data.distances(i) < data.distances(i-1) && data.distances(i) < data.distances(i+1) && data.distances(i) < value
                    local_min = i;
                    value = data.distances(i);
                end
            end

            if local_min ~= -1 % split and re-solve...
                % We need to find the right place to insert the x,y coordinate into the
                % original x,y data
                x_new = data.x_segment(local_min);
                y_new = data.y_segment(local_min);
                for i = 1:length(x)-1
                    if ((x(i) <= x(i+1) && x(i) <= x_new && x_new <= x(i+1)) || (x(i) >= x(i+1) && x(i) >= x_new && x_new >= x(i+1))) && ...
                            ((y(i) <= y(i+1) && y(i) <= y_new && y_new <= y(i+1)) || (y(i) >= y(i+1) && y(i) >= y_new && y_new >= y(i+1)))
                        break
                    end
                end
                x_I = [x(1:i); x_new];
                x_II = [x_new; x(i+1:end)];
                y_I = [y(1:i); y_new];
                y_II = [y_new; y(i+1:end)];
                if direction == 1 % first side
                    [k_I, ~] = splitHull(x_I, y_I);
                    [k_II, ~] = splitHull(x_II, y_II);
                else % second side
                    [~, k_I] = splitHull(x_I, y_I);
                    [~, k_II] = splitHull(x_II, y_II);
                end
                data_I = distanceFromHull(x_I, y_I, k_I, true);
                data_II = distanceFromHull(x_II, y_II, k_II, true);

                data.arc = [data_I.arc data_II.arc + data_I.arc(end)];
                data.distances = [data_I.distances data_II.distances];
                data.x_segment = [data_I.x_segment data_II.x_segment];
                data.y_segment = [data_I.y_segment data_II.y_segment];
                data.x_hull = [data_I.x_hull data_II.x_hull];
                data.y_hull = [data_I.y_hull data_II.y_hull];

                [data.arc I] = sort(data.arc);
                data.distances = data.distances(I);
                data.x_segment = data.x_segment(I);
                data.y_segment = data.y_segment(I);
                data.x_hull = data.x_hull(I);
                data.y_hull = data.y_hull(I);

                [data.arc I] = remove_duplicates(data.arc, data.distances);
                data.distances = data.distances(I);
                data.x_segment = data.x_segment(I);
                data.y_segment = data.y_segment(I);
                data.x_hull = data.x_hull(I);
                data.y_hull = data.y_hull(I);
            end
        end

        segment_position = [0];
        for i = 2:length(data.x_segment)
            segment_position(end+1) = segment_position(end) + sqrt((data.x_segment(i) - data.x_segment(i-1))^2 + (data.y_segment(i) - data.y_segment(i-1))^2);
        end

        data.segment_position = segment_position;
        [data.curva,data.bend_energy,data.slope]=curvature(data.x_segment,data.y_segment,5,5);
        data.distances100=com_dist(data.distances,100);
        data.curva100=com_dist(data.curva,100);
        data.bend_energy100=com_dist(data.bend_energy,100);
        data.max_dist=max(data.distances);
        data.hull_length=sqrt((data.x_hull(end)-data.x_hull(1))^2+(data.y_hull(end)-data.y_hull(1))^2);
        data.arc_length=0;
        for ii=2:length(data.x_segment)
            data.arc_length=data.arc_length+sqrt((data.x_segment(ii)-data.x_segment(ii-1))^2+(data.y_segment(ii)-data.y_segment(ii-1))^2);
        end
end
function [x, y] = interp_xy(X, Y, n)
% Returns interpolation of the paired data X, Y for any n in the range
% (integer or non-integer).
if mod(n, 1) == 0
    x = X(n);
    y = Y(n);
else
    n_1 = floor(n);
    n_2 = ceil(n);
    x = X(n_1) + (X(n_2) - X(n_1)) * mod(n, 1);
    y = Y(n_1) + (Y(n_2) - Y(n_1)) * mod(n, 1);
end
end
function [x I] = remove_duplicates(x, y)
    % Remove duplicate 'x' entries.  Chooses the lowest "y".

    I = 1:length(x);

    dupe_found = true;

    while dupe_found % this is an awful way to do this but it works fine
        dupe_found = false;
        for i_n = 1:length(I)
            i = I(i_n);
            for j_n = 1:length(I)
                j = I(j_n);
                if i ~= j && x(i) == x(j)
                    if y(i) < y(j)
                        I(j_n) = []; % delete the higher value
                    else
                        I(i_n) = [];
                    end
                    dupe_found = true;
                    break
                end
                if dupe_found
                    break
                end
            end
            if dupe_found
                break
            end
        end
    end

    x = x(I);

end
function k1=addconvhull(k1,num)
    if num==0
    else
        n=length(num);kn=length(k1);n0=n;
        for i=1:n
            for j=1:kn-1
                if num(i)>=k1(j)&&num(i)<k1(j+1)&&n0>0
                    k1=[k1(1:j);num(i);k1(j+1:end)];n0=n0-1;
                end
            end
        end
    end
end
function outs6=compare_z(outs3,outs6,n)
outs6.dist_fixz=ones(1,1);
for i=1:min(max(outs3.dist_fmark),max(outs6.dist_fmark))
    eval(['outs6.dist_fixz=[outs6.dist_fixz com_dist(outs3.side_' int2str(i) '_z' int2str(n) '.distances,size(find(outs6.dist_fmark==' int2str(i) '),2))];']);
    eval(['outs6.dist_fixz=[outs6.dist_fixz zeros(1,round(outs6.convex_length(2,' num2str(i) ')*10))];']);
end
outs6.dist_fixz=outs6.dist_fixz(2:end);
outs6.dist_fixz1000=com_dist(outs6.dist_fixz,1000);
end
function info=showparameter(p,sc)
info.pa={};
info.index={};

info.pa{1,1}='cellname';info.pa{1,2}=p.cellname;
info.pa{2,1}='replica';info.pa{2,2}=p.replica;
info.pa{3,1}='t1 stage';info.pa{3,2}=p.t1;
if isfield(p,'t2')==1
    info.pa{4,1}='t2 stage';info.pa{4,2}=p.t2;
end
if isfield(p,'lop')==1
info.lop{1,1}='expan_model';info.lop{1,2}=p.lop.expan_model;
info.lop{2,1}='exapnsion rate r';info.lop{2,2}=p.lop.r;
info.lop{3,1}='noise_p';info.lop{3,2}=p.lop.noise_p;
info.lop{4,1}='noise_height';info.lop{4,2}=p.lop.noise_height;
info.lop{5,1}='boundary noisy on-off';info.lop{5,2}=p.lop.noise_r;
info.lop{6,1}='built loop on-off';info.lop{6,2}=p.lop.lop_r;
info.lop{7,1}='lop_p_max';info.lop{7,2}=p.lop.lop_p_max;
info.lop{8,1}='lop_height_max';info.lop{8,2}=p.lop.lop_height_max;
info.lop{9,1}='lop_xy_max';info.lop{9,2}=p.lop.lop_xy_max;
end

if strcmp(sc,'s')
    info.index{1,1}='p';info.index{1,2}='oroginal convex points number(n)';
    info.index{2,1}='contP';info.index{2,2}='segments control points(x,y)';
    info.index{3,1}='outcontrol';info.index{3,2}='outter control points(2n,x,y)';
    info.index{4,1}='inncontrol';info.index{4,2}='inner control points(n,x,y)';
    info.index{5,1}='dist_fix';info.index{5,2}='x_asis trap_project convex hull distance';
    info.index{6,1}='dist_fmark';info.index{6,2}='x_asis mark trap_project convex hull distance';
    info.index{7,1}='dist_fix1000';info.index{7,2}='x_asis trap_project convex hull distance resize to 1000';
    info.index{8,1}='convex_length(1)';info.index{8,2}='concave length 3DAG-5DAG';
    info.index{9,1}='convex_length(2)';info.index{9,2}='convex length 3DAG-5DAG';
    info.index{10,1}='convex_length(3)';info.index{10,2}='segment Arc 3DAG-5DAG';
    info.index{11,1}='convex_length(4)';info.index{11,2}='segment MaxD 3DAG-5DAG';
    info.index{12,1}='convex_length_indi(1)';info.index{12,2}='concave/convex ratio';
    info.index{13,1}='convex_length_indi(2)';info.index{13,2}='hull/segment';
    info.index{14,1}='convex_length_indi(3)';info.index{14,2}='MaxD/hull';
    info.index{15,1}='convex_length_indi(4)';info.index{15,2}='MaxD/segment';
elseif strcmp(sc,'c')
    info.index{1,1}='x_cell';info.index{1,2}='cell point x after density adject';
    info.index{2,1}='y_cell';info.index{2,2}='cell point y after density adject';
    info.index{3,1}='x_hull';info.index{3,2}='hull point x after density adject';
    info.index{4,1}='y_hull';info.index{4,2}='hull point y after density adject';
    info.index{5,1}='distances';info.index{5,2}='distance of convex hull';
    info.index{6,1}='distnaces1000';info.index{6,2}='distance of convex hull after adject to 1000';
    info.index{7,1}='arc';info.index{7,2}='angle along the cell boundary';
    info.index{8,1}='cell_position';info.index{8,2}='ce position along cell boundary';
    info.index{9,1}='loop_range';info.index{9,2}='cell point x after density adject';
    info.index{10,1}='x';info.index{10,2}='original x positon before calculation';
    info.index{11,1}='y';info.index{11,2}='original y position before calculation';
    info.index{12,1}='maxpeak';info.index{12,2}='max distance in each segment (number distance)';
    info.index{13,1}='maxpeak1000';info.index{13,2}='max distance in each segment after adject 10000(number distance)';
    info.index{14,1}='curva';info.index{14,2}='curvature along cell boundary';
    info.index{15,1}='curva1000';info.index{15,2}='curvature along cell boundary after adject to 1000';
    info.index{16,1}='bend_energy';info.index{16,2}='bend_energy along cell boundary';
    info.index{17,1}='bend_energy1000';info.index{17,2}='bend_energy along cell boundary after adject 1000';
elseif strcmp(sc,'side')
    info.index{1,1}='x_segment';info.index{1,2}='segment point x after density adject(1xn)(segment)';
    info.index{2,1}='y_segment';info.index{2,2}='segment point y after density adject(1xn)(segment)';
    info.index{3,1}='x_hull';info.index{3,2}='hull point x after density adject(1xn)(segment)';
    info.index{4,1}='y_hull';info.index{4,2}='hull point y after density adject(1xn)(segment)';
    info.index{5,1}='distances';info.index{5,2}='distance of convex hull(1xn)(segment)';
    info.index{6,1}='distnaces100';info.index{6,2}='distance of convex hull after adject to 100(1xn)(segment)';
    info.index{7,1}='distances_x';info.index{7,2}='';
    info.index{8,1}='arc';info.index{8,2}='angle along cell boundary(1xn)(segment)';  
    info.index{9,1}='segment_position';info.index{9,2}='segment position(1xn)(segment)';
    info.index{10,1}='max_dist';info.index{10,2}='max distance in segment(1xn)(segment)';
    info.index{11,1}='hull_length';info.index{11,2}='hull length in segment(1xn)(segment)';
    info.index{12,1}='arc_length';info.index{12,2}='arc length in segment(1xn)(segment)';
    info.index{13,1}='peak';info.index{13,2}='max distance in each segment after adject 10000(number distance)';
    info.index{14,1}='peakmark';info.index{14,2}='max distance in each segment after adject 10000(number distance)';
    info.index{15,1}='curva';info.index{15,2}='curvature along cell boundary(1xn)(segment)';
    info.index{16,1}='curva100';info.index{16,2}='curvature along cell boundary after adject to 100(1xn)(segment)';
    info.index{17,1}='bend_energy';info.index{17,2}='bend_energy along cell boundary(1xn)(segment)';
    info.index{18,1}='bend_energy100';info.index{18,2}='bend_energy along cell boundary after adject 100(1xn)(segment)';
end

end
function outs=segment_info(outs)
maxi=-max(outs.dist_fmark);
outs.convex_length_indi=zeros(4,-maxi);
for i=1:-maxi;
    if i==1
        outs.side_1.peak=[outs.dist_fix(outs.dist_fmark==maxi) outs.dist_fix(outs.dist_fmark==1) outs.dist_fix(outs.dist_fmark==-1)];
        outs.side_1.peakmark=[outs.dist_fmark(outs.dist_fmark==maxi) outs.dist_fmark(outs.dist_fmark==1) outs.dist_fmark(outs.dist_fmark==-1)];
        outs.convex_length_indi(1,i)=outs.convex_length(1,i)/(outs.convex_length(1,i)+outs.convex_length(2,-maxi)+outs.convex_length(2,i));
        outs.convex_length_indi(2,i)=outs.convex_length(1,i)/outs.convex_length(3,i);
        outs.convex_length_indi(3,i)=outs.convex_length(4,i)/outs.convex_length(1,i);
        outs.convex_length_indi(4,i)=outs.convex_length(4,i)/outs.convex_length(3,i);
    else
        eval(['outs.side_' int2str(i) '.peak=[outs.dist_fix(outs.dist_fmark==-(' int2str(i) '-1)) outs.dist_fix(outs.dist_fmark==' int2str(i) ') outs.dist_fix(outs.dist_fmark==-' int2str(i) ')];']);
        eval(['outs.side_' int2str(i) '.peakmark=[outs.dist_fmark(outs.dist_fmark==-(' int2str(i) '-1)) outs.dist_fmark(outs.dist_fmark==' int2str(i) ') outs.dist_fmark(outs.dist_fmark==-' int2str(i) ')];']);
        outs.convex_length_indi(1,i)=outs.convex_length(1,i)/(outs.convex_length(1,i)+outs.convex_length(2,i-1)+outs.convex_length(2,i));
        outs.convex_length_indi(2,i)=outs.convex_length(1,i)/outs.convex_length(3,i);
        outs.convex_length_indi(3,i)=outs.convex_length(4,i)/outs.convex_length(1,i);
        outs.convex_length_indi(4,i)=outs.convex_length(4,i)/outs.convex_length(3,i);
    end
end
end
function length=cal_radius(input)
%ex: perimeter=cal_dist([input]nx2 matrix)

[n1,n2]=size(input);
if n2>2&&n1==2
    input=input';
end
length=0;
for i=1:n1
    length=length+distance(0,0,input(i,1),input(i,2));
    length=length+sqrt(input(i,1)^2+input(i,2)^2)/p.pixel_di_um;
end
length=length/n1;
end
function length=cal_dist(input,p)
%ex: perimeter=cal_dist([input]nx2 matrix)

[n1,n2]=size(input);
if n2>2&&n1==2
    input=input';
end
input=[input;input(1,:)];
length=0;
for i=2:size(input,1)
    length=length+(sqrt((input(i,1)-input(i-1,1))^2+(input(i,2)-input(i-1,2))^2))/p.pixel_di_um;
end
end
function theta=xytotheta(xxx,yyy)
% xxx=(1xn), yyy=[1xn]
n=size(xxx,2);theta=zeros(1,n);
for kk=1:n
    xx=xxx(1,kk);yy=yyy(1,kk);
    if xx>=0&&yy>=0
        theta(1,kk)=atan(abs(yy)/abs(xx));
    elseif xx<0&&yy>=0
        theta(1,kk)=pi-atan(abs(yy)/abs(xx));
    elseif xx<0&&yy<0
        theta(1,kk)=pi+atan(abs(yy)/abs(xx));
    elseif  xx>=0&&yy<0
        theta(1,kk)=2*pi-atan(abs(yy)/abs(xx));
    end
end
end
function [Lb_Sensitivity, Lb_Specificity, Lb_Accuracy, Lb_FDR, lb_result]=lobe_position_result(algorithm_lobe_position,data_lobe_position,data_length,correct_threa)
a_n=size(algorithm_lobe_position,2);d_pp=size(data_lobe_position,1);
lb_result=zeros(2,2);
total_n=data_length;
for j=1:d_pp
    AA=[];AAA=0;
    cal_yes=0;
    d_n=size(data_lobe_position{j,1},2);
    for i=1:d_n
        %A=[];
        lobe_ans=data_lobe_position{j,1}(1,i);
        %[~,A]=min(abs(lobe_ans-algorithm_lobe_position(1,:)));
        A=find(algorithm_lobe_position(1,:)>=lobe_ans-correct_threa&algorithm_lobe_position(1,:)<=lobe_ans+correct_threa);
        %if isempty(B)
        %    A=[];
        %else
        AA=[AA A];
        %end
        if isempty(A)
            lb_result(2,1)=lb_result(2,1)+1;
        else
            lb_result(1,1)=lb_result(1,1)+1;
            cal_yes=cal_yes+1;
        end
    end
    for ii=1:d_n
        if isempty(find(AA==ii))
            AAA=AAA+1;
        end
    end
    lb_result(1,2)=lb_result(1,2)+AAA;
    lb_result(2,2)=lb_result(2,2)+total_n-d_n-AAA;
end
Lb_Sensitivity=lb_result(1,1)/(lb_result(1,1)+lb_result(2,1));
Lb_Specificity=lb_result(2,2)/(lb_result(1,2)+lb_result(2,2));
Lb_FDR=lb_result(1,2)/(lb_result(1,1)+lb_result(1,2));
Lb_Accuracy=(lb_result(1,1)+lb_result(2,2))/(lb_result(1,1)+lb_result(1,2)+lb_result(2,1)+lb_result(2,2));
end
function cell = ProcessCell_withoutpartial(point_before, p)
% 1. Input cell(X,Y) and make cell convex hull.
% 2. Calculate curva and bending curve.

% Parameter deliver
cellname=p.cellname;
replica=p.replica;
adjust_hull='';
section_number=p.section_number;
d=p.d;
radius=p.radius;

% move to center and resize by r_x, r_y
[out2,ratio_r,center,first_point]=movC_resize(point_before(:,1:2)',zeros(2,1),p); 

% interpolation by spline2
points=spline2(out2',d,section_number);

% points corresponding junction points
points_row3=zeros(size(points,1),1);
if size(point_before,2)<3
    point_before=[point_before zeros(size(point_before,1),1)];
else    
    point_before_1=find(point_before(:,3)==1);point_before_1=point_before_1-first_point+1;
    for jj=1:size(point_before_1)
       if point_before_1(jj)>size(out2,2)
           point_before_1(jj)=point_before_1(jj)-size(out2,2);
       elseif point_before_1(jj)<1
           point_before_1(jj)=point_before_1(jj)+size(out2,2);
       end
    end
    A=out2(:,point_before_1);
    for ii=1: size(A,2)
        [~,a]=min(distance(A(:,ii)',points));
        points_row3(a,1)=1;
    end
end
points=[points points_row3];

% pre-setting points
if size(points,2)>3
    points=points';
end
%[points,center]=movC(points',zeros(2,1));points=points';  
x = points(:,1)';y = points(:,2)';
x_o=x/ratio_r;y_o=y/ratio_r;

meanr=0;meanr_o=0;
for jj=1:size(x,2)-1
   meanr=meanr+sqrt(x(jj)^2+y(jj)^2);
   meanr_o=meanr_o+sqrt(x_o(jj)^2+y_o(jj)^2);
end
p.meanr=meanr/(size(x,2)-1);p.meanr_o=meanr_o/(size(x,2)-1);

k = convhull(x, y);
k=convhull_redirect(k);
cell = distanceFromHull(x, y, k, adjust_hull);
cell.xy = [x;y;points(:,3)'];cell.xy_o = [x_o;y_o;points(:,3)'];
cell.area=polyarea(cell.x_cell/ratio_r',cell.y_cell/ratio_r');
cell.C_area=polyarea(cell.x_hull/ratio_r',cell.y_hull/ratio_r');
%cell.perimeter=cal_dist([cell.x_cell/ratio_r;cell.y_cell/ratio_r]');
cell.perimeter=cal_dist([cell.xy(1,:)/ratio_r;cell.xy(2,:)/ratio_r]');
cell.C_perimeter=cal_dist([cell.x_hull/ratio_r;cell.y_hull/ratio_r]');
cell.mean_radius=cal_radius([cell.x_hull/ratio_r;cell.y_hull/ratio_r]');
[cell.klobe(:,1),cell.klobe(:,2)]=double_convhull(x,y,cell,p,k);
[cell.klobe_e(:,1),cell.klobe_e(:,2)]=extract_lobe(cell.klobe,cell.xy);
cell.klobe_num=max(cell.klobe(:,2));
cell.ratio_r=ratio_r;

% klobe_position
cell.klobe_position=zeros(3,cell.klobe_num);
for kk=1:cell.klobe_num
    xx=cell.xy_o(1,cell.klobe_e(kk,1));yy=cell.xy_o(2,cell.klobe_e(kk,1));
    if xx>=0&&yy>=0
        cell.klobe_position(1,kk)=atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif xx<0&&yy>=0
        cell.klobe_position(1,kk)=pi-atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif xx<0&&yy<0
        cell.klobe_position(1,kk)=pi+atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    elseif  xx>=0&&yy<0
        cell.klobe_position(1,kk)=2*pi-atan(abs(yy)/abs(xx));
        cell.klobe_position(2,kk)=xx;cell.klobe_position(3,kk)=yy;
    end
end

cell.info=showparameter(p,'c');
if any(strcmp('cen',fieldnames(p)))==1
    n_pp=size(p.cen,1);cexy=[x_o' y_o'];
    for iii=1:n_pp
        for iij=1:size(p.cen{iii,1},2)             
            p.ce{iii,1}(:,iij)=p.cen{iii,1}(1,iij);
        end
    end
end
if any(strcmp('ce',fieldnames(p)))==1
    n_pp=size(p.ce,1);cell.lobe_number_voting=zeros(1,n_pp);
    for pp=1:n_pp
        cell.lobe_number_voting(1,pp)=size(p.ce{pp,1},2);
    end
    [cell.Lb_Sensitivity, cell.Lb_Specificity, cell.Lb_Accuracy, cell.Lb_FDR, cell.lb_result]=lobe_position_result(cell.klobe_position(1,:),p.cen,size(point_before,1),p.correct_threa);
end
cell.compactness=4*pi*cell.area/(cell.perimeter)^2;
cell.roundness=4*pi*cell.area/(cell.C_perimeter)^2;
cell.convexity=cell.C_perimeter/cell.perimeter;
cell.solidity=cell.area/cell.C_area;
cell.p=p;

    function [k_I k_II] =  splitHull(x, y)
        % Find the control points for the convex hull and split it into the hulls
        % for each side.

        % find convex hull
        k = convhull(x, y);k=convhull_redirect(k);
        % split the hull into two sides
        k_I = [];
        k_II = [];
        k_last = -inf; %k(1);
        for k_i = k'
            if k_i > k_last
                k_I(end + 1) = k_i; 
            elseif k_i < k_last
                k_II(end + 1) = k_i;
            end
            k_last = k_i;
        end
        k_II = [k_I(end) k_II];
    end
    function data = distanceFromHull(x, y, k, adjust_hull)
        data = struct;

        %direction = 2*(k(2) > k(1))-1; % needed for the iterative part below
        %k = sort(k);

        data.arc = [];
        data.distances = [];
        data.x_cell = [];
        data.y_cell = [];
        data.x_hull = [];
        data.y_hull = [];
        d = 0.5;
        % d is the density of points sampled; 0.1 works well but may appear jagged

        % break the hull into bite-size pieces for analysis
        for f = 1:length(k)-1
            if isempty(data.arc) % arc_last tracks the last position along the hull so that lengths are additive
                arc_last = 0;
            else
                arc_last = data.arc(end);
            end

            k_1 = k(f);
            k_2 = k(f + 1);

            % create dense arrays of points on the hull
            hull_x = [];
            hull_y = [];
            hull_pos = [];
            distance = sqrt((x(k_1) - x(k_2)).^2 + (y(k_1) - y(k_2)).^2);
            for i = linspace(1, 2, ceil(distance/d))
                [hull_x(end + 1), hull_y(end + 1)] = interp_xy(x([k_1 k_2]), y([k_1 k_2]), i);
                if isempty(hull_pos)
                    hull_pos(end + 1) = 0;
                else
                    hull_pos(end + 1) = hull_pos(end) + sqrt((hull_x(end) - hull_x(end - 1)).^2 + (hull_y(end) - hull_y(end - 1)).^2 );
                end
            end

            % create dense arrays of points on the segment
            wall_x = [];
            wall_y = [];
            if k_2 > k_1
                for i = k_1:k_2-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
            else
                for i = k_1:length(x)-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
                for i = 1:k_2-1
                    p_1 = i;
                    p_2 = i + 1;
                    distance = sqrt((x(p_1) - x(p_2)).^2 + (y(p_1) - y(p_2)).^2);
                    for j = linspace(p_1, p_2, ceil(distance/d))
                        [wall_x(end + 1), wall_y(end + 1)] = interp_xy(x, y, j);
                    end
                end
                %disp(k_1)
                %disp(k_2)
                %disp('-----')
            end

            % find the closest point from a point on the hull to the segment
            for i = 1:length(wall_x)
                wx = wall_x(i);
                wy = wall_y(i);
                dist = inf;
                for j = 1:length(hull_x)
                    hx = hull_x(j);
                    hy = hull_y(j);
                    di = sqrt((wx - hx).^2 + (wy - hy).^2);
                    if di < dist
                        dist = di;
                        best = j;
                    end
                end
                if dist < inf
                    data.distances(end + 1) = dist; % record distance between points
                    data.arc(end + 1) = arc_last + hull_pos(best); % record distance along arc
                    data.x_cell(end + 1) = wx;
                    data.y_cell(end + 1) = wy;
                    data.x_hull(end + 1) = hull_x(best);
                    data.y_hull(end + 1) = hull_y(best);
                end
            end
        end

        [data.arc I] = sort(data.arc);
        data.distances = data.distances(I);
        data.x_cell = data.x_cell(I);
        data.y_cell = data.y_cell(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);

        [data.arc I] = remove_duplicates(data.arc, data.distances);
        data.distances = data.distances(I);
        data.x_cell = data.x_cell(I);
        data.y_cell = data.y_cell(I);
        data.x_hull = data.x_hull(I);
        data.y_hull = data.y_hull(I);

        %{
        if adjust_hull

            % Search for local minima (disregarding values < the sampling distance, d).
            % If any are found, clean out arc and distances, record the x/y coordinate,
            % insert a new control point for the "hull" and recalculate the
            % distance-from-hull for either side.  Then recombine the results.

            local_min = -1;
            value = inf;

            for i = 2:length(data.arc)-1
                if data.distances(i) > d && data.distances(i) < data.distances(i-1) && data.distances(i) < data.distances(i+1) && data.distances(i) < value
                    local_min = i;
                    value = data.distances(i);
                end
            end

            if local_min ~= -1 % split and re-solve...
                % We need to find the right place to insert the x,y coordinate into the
                % original x,y data
                x_new = data.x_cell(local_min);
                y_new = data.y_cell(local_min);
                for i = 1:length(x)-1
                    if ((x(i) <= x(i+1) && x(i) <= x_new && x_new <= x(i+1)) || (x(i) >= x(i+1) && x(i) >= x_new && x_new >= x(i+1))) && ...
                            ((y(i) <= y(i+1) && y(i) <= y_new && y_new <= y(i+1)) || (y(i) >= y(i+1) && y(i) >= y_new && y_new >= y(i+1)))
                        break
                    end
                end
                x_I = [x(1:i) x_new];
                x_II = [x_new x(i+1:end)];
                y_I = [y(1:i) y_new];
                y_II = [y_new y(i+1:end)];
                if length(x_I) >= 2 && length(x_II) >= 2 && (x_I(1) ~= x_I(2) || y_I(1) ~= y_I(2)) && (x_II(1) ~= x_II(2) || y_II(1) ~= y_II(2))
                    k_I = convhull(x_I, y_I);%k=convhull_redirect(k);
                    k_II = convhull(x_II, y_II);%k=convhull_redirect(k);
                    data_I = distanceFromHull(x_I, y_I, k_I, true);
                    data_II = distanceFromHull(x_II, y_II, k_II, true);

                    data.arc = [data_I.arc data_II.arc + data_I.arc(end)];
                    data.distances = [data_I.distances data_II.distances];
                    data.x_cell = [data_I.x_cell data_II.x_cell];
                    data.y_cell = [data_I.y_cell data_II.y_cell];
                    data.x_hull = [data_I.x_hull data_II.x_hull];
                    data.y_hull = [data_I.y_hull data_II.y_hull];

                    [data.arc I] = sort(data.arc);
                    data.distances = data.distances(I);
                    data.x_cell = data.x_cell(I);
                    data.y_cell = data.y_cell(I);
                    data.x_hull = data.x_hull(I);
                    data.y_hull = data.y_hull(I);

                    [data.arc I] = remove_duplicates(data.arc, data.distances);
                    data.distances = data.distances(I);
                    data.x_cell = data.x_cell(I);
                    data.y_cell = data.y_cell(I);
                    data.x_hull = data.x_hull(I);
                    data.y_hull = data.y_hull(I);
                end
            end
        end
%}
        cell_position = [0];
        for i = 2:length(data.x_cell)
            cell_position(end+1) = cell_position(end) + sqrt((data.x_cell(i) - data.x_cell(i-1))^2 + (data.y_cell(i) - data.y_cell(i-1))^2);
        end

        data.cell_position = cell_position;
        data.distances1000=com_dist(data.distances,1000);
    end
    function [x, y] = interp_xy(X, Y, n)
        % Returns interpolation of the paired data X, Y for any n in the range
        % (integer or non-integer).
        if mod(n, 1) == 0
            x = X(n);
            y = Y(n);
        else
            n_1 = floor(n);
            n_2 = ceil(n);
            x = X(n_1) + (X(n_2) - X(n_1)) * mod(n, 1);
            y = Y(n_1) + (Y(n_2) - Y(n_1)) * mod(n, 1);
        end
    end
    function [x I] = remove_duplicates(x, y)
        % Remove duplicate 'x' entries.  Chooses the lowest "y".

        I = 1:length(x);

        dupe_found = true;
        i_last = 0;
        while dupe_found % this is an awful way to do this but it works fine
            dupe_found = false;
            for i_n = i_last+1:length(I)
                i = I(i_n);
                for j_n = i_n+1:length(I)
                    j = I(j_n);
                    if i ~= j && x(i) == x(j)
                        if y(i) < y(j)
                            I(j_n) = []; % delete the higher value
                        else
                            I(i_n) = [];
                        end
                        i_last = i;
                        dupe_found = true;
                        break
                    end
                    if dupe_found
                        break
                    end
                end
                if dupe_found
                    break
                end
            end
        end

        x = x(I);
    end
    function [newk,newkloop]=double_convhull(x,y,cell,p,k)
       if exist('k')==0
           k = convhull(x, y);k=convhull_redirect(k);
       end
       newkloop=convex2loop(x,y,k,p);newk=k;
       %[newk,newkloop]=second_convex(x,y,cell,k,newkloop,p);
    end
    function [newk,newkloop]=second_convex(x,y,cell,k,kloop,p)
        % second convex(search inner points)
        
        % use partial_convexhull to track newkk, and added to k2
        k2=k;newkk=[];
        for q=2:size(kloop,1)
            if abs(kloop(q)-kloop(q-1))>0
                newkk=partial_convhull(x,y,cell,k(q-1),k(q),p);
            end
            if isempty(newkk)==1
            else
                k2=[k2;newkk];
                newkk=[];
            end
        end
        
        % add new k2 into newk
        k3=[];
        sort(k2);k2=[k2;k2(1)];
        for i=1:size(x,2)
            if find(k2==i)
                k3=[k3;i];
            end
        end
        newk=[k3;k3(1)];
        
        % run convex2loop again to find new newkloop
        newkloop=convex2loop(x,y,newk,p);newkloop(newkloop==max(newkloop))=1;
    end
    function newk=partial_convhull(x,y,cell,startk,endk,p)
    [~,start_convd]=min((x(startk)-cell.x_cell).^2+(y(startk)-cell.y_cell).^2); % find the corresponding starting point on HODC distance from k 
    [~,end_convd]=min((x(endk)-cell.x_cell).^2+(y(endk)-cell.y_cell).^2); % find the corresponding end point on DOCH distance from k
    newk=[];
    if start_convd<end_convd
        Convex_dist=cell.distances(1,start_convd:end_convd); % specific region DOCH 
    elseif (start_convd>end_convd)&&(start_convd-end_convd<=0.5*size(cell.x_cell,2))
        Convex_dist=cell.distances(1,start_convd:-1:end_convd); % specific region DOCH(inverse direction)
    else
        Convex_dist=cell.distances(1,[start_convd:size(cell.distances,2) 1:end_convd]); % specific region DOCH across(0,0)
    end
    if distance(x(startk),y(startk),x(endk),y(endk))>p.gap_k | max(Convex_dist)/distance(x(startk),y(startk),x(endk),y(endk))>p.h_width_ratio
        varargout = peakfinder(Convex_dist,(max(Convex_dist)-min(Convex_dist))/p.sel,max(Convex_dist),-1); % find the local min points on Convex_dist
        varargout=[varargout diff2(Convex_dist,p.diff2_n,p.diff2_thread)];
        [~,newk_n]=find(varargout~=1&varargout~=size(Convex_dist,2));
        if isempty(newk_n)~=1
            newk_xy=varargout(newk_n);nn=size(newk_xy,2);
            for i=1:nn
                if start_convd<end_convd  % specific region DOCH
                    [~,newkk]=min((cell.x_cell(newk_xy(i)+start_convd-1)-x).^2+(cell.y_cell(newk_xy(i)+start_convd-1)-y).^2);
                    newkkk=newkk;
                elseif (start_convd>end_convd)&&(start_convd-end_convd<=0.5*size(cell.x_cell,2))  % specific region DOCH(inverse direction)
                    [~,newkk]=min((cell.x_cell(start_convd-newk_xy(i)+1)-x).^2+(cell.y_cell(start_convd-newk_xy(i)+1)-y).^2);
                    newkkk=newkk;
            else  % specific region DOCH across(0,0)
                if newk_xy(i)<=(size(cell.x_cell,2)-start_convd)
                    [~,newkk]=min((cell.x_cell(newk_xy(i)+start_convd-1)-x).^2+(cell.y_cell(newk_xy(i)+start_convd-1)-y).^2);
                    newkkk=newkk;
                else
                    [~,newkk]=min((cell.x_cell(newk_xy(i)-size(cell.x_cell,2)+start_convd)-x).^2+(cell.y_cell(newk_xy(i)-size(cell.x_cell,2)+start_convd)-y).^2);
                    newkkk=newkk;
                end
            end
                newk=[newk;newkkk];
            end
        end
    end
    end
    function [mi_distance mi_p m2]=distance_p2line(line1,line2,points)
        % line1(start point)=(x1, y1), line2(end point)=(x2,y2)
        % points=[p1;p2;p3;p4.....];
        div=100;
        xspace=linspace(line1(1),line2(1),div);
        yspace=linspace(line1(2),line2(2),div);
        n=size(points,1);mi_distxy=zeros(n,1);
        for i=1:n
            mi_d=1000000;
            for j=1:div
            mi_d=min(mi_d,distance([xspace(j) yspace(j)],points(i,:)));
            end
            mi_distxy(i,1)=mi_d;
        end
        [mi_distance,m2]=max(mi_distxy);mi_p=points(m2,:);
    end
    function varargout = peakfinder(x0, sel, thresh, extrema)
        %PEAKFINDER Noise tolerant fast peak finding algorithm
        %   INPUTS:
        %       x0 - A real vector from the maxima will be found (required)
        %       sel - The amount above surrounding data for a peak to be
        %           identified (default = (max(x0)-min(x0))/4). Larger values mean
        %           the algorithm is more selective in finding peaks.
        %       thresh - A threshold value which peaks must be larger than to be
        %           maxima or smaller than to be minima.
        %       extrema - 1 if maxima are desired, -1 if minima are desired
        %           (default = maxima, 1)
        %   OUTPUTS:
        %       peakLoc - The indicies of the identified peaks in x0
        %       peakMag - The magnitude of the identified peaks
        %
        %   [peakLoc] = peakfinder(x0) returns the indicies of local maxima that
        %       are at least 1/4 the range of the data above surrounding data.
        %
        %   [peakLoc] = peakfinder(x0,sel) returns the indicies of local maxima
        %       that are at least sel above surrounding data.
        %
        %   [peakLoc] = peakfinder(x0,sel,thresh) returns the indicies of local 
        %       maxima that are at least sel above surrounding data and larger
        %       (smaller) than thresh if you are finding maxima (minima).
        %
        %   [peakLoc] = peakfinder(x0,sel,thresh,extrema) returns the maxima of the
        %       data if extrema > 0 and the minima of the data if extrema < 0
        %
        %   [peakLoc, peakMag] = peakfinder(x0,...) returns the indicies of the
        %       local maxima as well as the magnitudes of those maxima
        %
        %   If called with no output the identified maxima will be plotted along
        %       with the input data.
        %
        %   Note: If repeated values are found the first is identified as the peak
        %
        % Ex:
        % t = 0:.0001:10;
        % x = 12*sin(10*2*pi*t)-3*sin(.1*2*pi*t)+randn(1,numel(t));
        % x(1250:1255) = max(x);
        % peakfinder(x)
        %
        % Copyright Nathanael C. Yoder 2011 (nyoder@gmail.com)

        % Perform error checking and set defaults if not passed in
        error(nargchk(1,4,nargin,'struct'));
        error(nargoutchk(0,2,nargout,'struct'));

        s = size(x0);
        flipData =  s(1) < s(2);
        len0 = numel(x0);
        if len0 ~= s(1) && len0 ~= s(2)
            error('PEAKFINDER:Input','The input data must be a vector')
        elseif isempty(x0)
            varargout = {[],[]};
            return;
        end
        if ~isreal(x0)
            warning('PEAKFINDER:NotReal','Absolute value of data will be used')
            x0 = abs(x0);
        end

        if nargin < 2 || isempty(sel)
            sel = (max(x0)-min(x0))/4;
        elseif ~isnumeric(sel) || ~isreal(sel)
            sel = (max(x0)-min(x0))/4;
            warning('PEAKFINDER:InvalidSel',...
                'The selectivity must be a real scalar.  A selectivity of %.4g will be used',sel)
        elseif numel(sel) > 1
            warning('PEAKFINDER:InvalidSel',...
                'The selectivity must be a scalar.  The first selectivity value in the vector will be used.')
            sel = sel(1);
        end

        if nargin < 3 || isempty(thresh)
            thresh = [];
        elseif ~isnumeric(thresh) || ~isreal(thresh)
            thresh = [];
            warning('PEAKFINDER:InvalidThreshold',...
                'The threshold must be a real scalar. No threshold will be used.')
        elseif numel(thresh) > 1
            thresh = thresh(1);
            warning('PEAKFINDER:InvalidThreshold',...
                'The threshold must be a scalar.  The first threshold value in the vector will be used.')
        end

        if nargin < 4 || isempty(extrema)
            extrema = 1;
        else
            extrema = sign(extrema(1)); % Should only be 1 or -1 but make sure
            if extrema == 0
                error('PEAKFINDER:ZeroMaxima','Either 1 (for maxima) or -1 (for minima) must be input for extrema');
            end
        end

        x0 = extrema*x0(:); % Make it so we are finding maxima regardless
        thresh = thresh*extrema; % Adjust threshold according to extrema.
        dx0 = diff(x0); % Find derivative
        dx0(dx0 == 0) = -eps; % This is so we find the first of repeated values
        ind = find(dx0(1:end-1).*dx0(2:end) < 0)+1; % Find where the derivative changes sign

        % Include endpoints in potential peaks and valleys
        x = [x0(1);x0(ind);x0(end)];
        ind = [1;ind;len0];

        % x only has the peaks, valleys, and endpoints
        len = numel(x);
        minMag = min(x);


        if len > 2 % Function with peaks and valleys

            % Set initial parameters for loop
            tempMag = minMag;
            foundPeak = false;
            leftMin = minMag;

            % Deal with first point a little differently since tacked it on
            % Calculate the sign of the derivative since we taked the first point
            %  on it does not neccessarily alternate like the rest.
            signDx = sign(diff(x(1:3)));
            if signDx(1) <= 0 % The first point is larger or equal to the second
                ii = 0;
                if signDx(1) == signDx(2) % Want alternating signs
                    x(2) = [];
                    ind(2) = [];
                    len = len-1;
                end
            else % First point is smaller than the second
                ii = 1;
                if signDx(1) == signDx(2) % Want alternating signs
                    x(1) = [];
                    ind(1) = [];
                    len = len-1;
                end
            end

            % Preallocate max number of maxima
            maxPeaks = ceil(len/2);
            peakLoc = zeros(maxPeaks,1);
            peakMag = zeros(maxPeaks,1);
            cInd = 1;
            % Loop through extrema which should be peaks and then valleys
            while ii < len
                ii = ii+1; % This is a peak
                % Reset peak finding if we had a peak and the next peak is bigger
                %   than the last or the left min was small enough to reset.
                if foundPeak
                    tempMag = minMag;
                    foundPeak = false;
                end

                % Make sure we don't iterate past the length of our vector
                if ii == len
                    break; % We assign the last point differently out of the loop
                end

                % Found new peak that was lager than temp mag and selectivity larger
                %   than the minimum to its left.
                if x(ii) > tempMag && x(ii) > leftMin + sel
                    tempLoc = ii;
                    tempMag = x(ii);
                end

                ii = ii+1; % Move onto the valley
                % Come down at least sel from peak
                if ~foundPeak && tempMag > sel + x(ii)
                    foundPeak = true; % We have found a peak
                    leftMin = x(ii);
                    peakLoc(cInd) = tempLoc; % Add peak to index
                    peakMag(cInd) = tempMag;
                    cInd = cInd+1;
                elseif x(ii) < leftMin % New left minima
                    leftMin = x(ii);
                end
            end

            % Check end point
            if x(end) > tempMag && x(end) > leftMin + sel
                peakLoc(cInd) = len;
                peakMag(cInd) = x(end);
                cInd = cInd + 1;
            elseif ~foundPeak && tempMag > minMag % Check if we still need to add the last point
                peakLoc(cInd) = tempLoc;
                peakMag(cInd) = tempMag;
                cInd = cInd + 1;
            end

            % Create output
            peakInds = ind(peakLoc(1:cInd-1));
            peakMags = peakMag(1:cInd-1);
        else % This is a monotone function where an endpoint is the only peak
            [peakMags,xInd] = max(x);
            if peakMags > minMag + sel
                peakInds = ind(xInd);
            else
                peakMags = [];
                peakInds = [];
            end
        end

        % Apply threshold value.  Since always finding maxima it will always be
        %   larger than the thresh.
        if ~isempty(thresh)
            m = peakMags>thresh;
            peakInds = peakInds(m);
            peakMags = peakMags(m);
        end



        % Rotate data if needed
        if flipData
            peakMags = peakMags.';
            peakInds = peakInds.';
        end



        % Change sign of data if was finding minima
        if extrema < 0
            peakMags = -peakMags;
            x0 = -x0;
        end
        % Plot if no output desired
        if nargout == 0
            if isempty(peakInds)
                disp('No significant peaks found')
            else
                figure;
                plot(1:len0,x0,'.-',peakInds,peakMags,'ro','linewidth',2);
            end
        else
            varargout = {peakInds,peakMags};
        end
    end
    function k=diff2(in,n,thread)
        % to calculate second order differential value
        
        % outt = moving average(15) of in.
        k=[];
        nn=fix(size(in,2)/n);
        outt=zeros(1,nn);
        for i=1:nn
           outt(1,i)=mean(in((i-1)*n+1:i*n)); 
        end
        
        % obtain the second order difference of outt
        out=diff(outt,2); % 2nd diff
        [temp,b]=find(out>=thread);
        if isempty(b)~=1
            for j=1:size(b,2)
                [temp,bb]=max(diff(in(b(1,j)*n+1:(b(1,j)+1)*n),2));
                k=[k b(1,j)*n+1+bb];
            end
        end
    end
    function [klobe1,klobe2]=extract_lobe(klobe,xy)
        klobe1=zeros(max(klobe(:,2)),1);klobe2=zeros(max(klobe(:,2)),1);
        for i=1:max(klobe(:,2))
            se1=klobe(klobe(:,2)==i,1);
            if size(se1,1)==1
                klobe1(i)=klobe(klobe(:,2)==i,1);klobe2(i)=klobe(klobe(:,2)==i,2);
            elseif size(se1,1)>1 %&&i~=1
                if isempty(find(xy(3,se1(1):se1(end))==1, 1))~=1
                    [~,~, m2]=distance_p2line(xy(1:2,se1(1))',xy(1:2,se1(end))',xy(1:2,se1(1):se1(end))');
                    nn=find(xy(3,se1(1):se1(end))==1);
                    if abs(nn(1)-m2)<2
                        klobe1(i)=se1(1)+nn(1)-1;klobe2(i)=i;
                    else
                        klobe1(i)=m2+se1(1)-1;klobe2(i)=i;
                    end
                else
                    [mi_distance mi_p m2]=distance_p2line(xy(1:2,se1(1))',xy(1:2,se1(end))',xy(1:2,se1(1):se1(end))');
                    %[curva,~]=curvature(xy(1,se1(1):se1(end)),xy(2,se1(1):se1(end)),min(5,size(se1,1)));[~,m2]=max(curva);
                    klobe1(i)=m2+se1(1)-1;klobe2(i)=i;
                end
            elseif i==1
                temp=find(klobe(:,2)==i);aa=min(temp);
                klobe1(i)=aa; klobe2(i)=1;
            end
        end
    end
    function kloop=convex2loop(x,y,k,p)
        %if k(end)==k(1)
        %   k(end)=size(x,2); 
        %end
        kloop=zeros(size(k,1),size(k,2));
        loop_marker=1;kloop(1)=loop_marker;
        for i=2:size(k,1)
            if k(i)-k(i-1)>=0
                points=[x(k(i-1):k(i))' y(k(i-1):k(i))'];
            elseif k(i-1)-k(i)<size(x,2)*0.5
                points=[x(k(i-1):-1:k(i))' y(k(i-1):-1:k(i))'];
            else
                points=[x([k(i-1):end 1:k(i)])' y([k(i-1):end 1:k(i)])'];
            end
                mi_distance=distance_p2line([x(k(i-1)) y(k(i-1))],[x(k(i)) y(k(i))],points);   
            if distance(x(k(i)),y(k(i)),x(k(i-1)),y(k(i-1)))>p.gap_k|mi_distance/distance([x(k(i-1)) y(k(i-1))],[x(k(i)) y(k(i))])>p.h_width_ratio
                    loop_marker=loop_marker+1;
            end
            kloop(i)=loop_marker;
        end
    end
    function newk=convhull_redirect(k)
        ord=diff(k);[a,~]=size(k);
        posi=numel(find(ord>0));negi=numel(find(ord<0));
        if posi>negi
            newk=k;
        elseif posi<negi
            newk=zeros(a,1);
            for i=1:a
                newk(i,1)=k(a-i+1,1);
            end
        end
    end
        
end
function xls_delete_sheets(xlsfile,sheets)
% 
% Delete worksheets in Excel file
% 
% 
%USAGE
%-----
% xls_delete_sheets(xlsfile)
% xls_delete_sheets(xlsfile,sheets)
% 
% 
%INPUT
%-----
% - XLSFILE: name of the Excel file
% - SHEETS : cell array with the worksheet names, or matrix with positive
%   integers to tell which worksheets are going to be protected. If
%   SHEETS=[], all empty sheets will be deleted.
% 
% 
%OUTPUT
%------
% - XLSFILE will be edited
% 
% 
% Based on "How do I delete worksheets in my Excel file using MATLAB?"
% (http://www.mathworks.com/support/solutions/en/data/1-21EPB4/index.html?solution=1-21EPB4)
% 
% 
% See also XLS_CHECK_IF_OPEN
% 

% Guilherme Coco Beltramini (guicoco@gmail.com)
% 2012-Dec-30, 05:29 pm


% Input
%==========================================================================
if exist(xlsfile,'file')~=2
    fprintf('%s not found.\n',xlsfile)
    return
end
if nargin<2
    sheets = [];
end


% Close Excel file
%-----------------
tmp = xls_check_if_open(xlsfile,'close');
if tmp~=0 && tmp~=10
    fprintf('%s could not be closed.\n',xlsfile)
    return
end


% The full path is required for the command "workbooks.Open" to work
% properly
%-------------------------------------------------------------------
if isempty(strfind(xlsfile,filesep))
    xlsfile = fullfile(pwd,xlsfile);
end


% Read Excel file
%==========================================================================
%[type,sheet_names] = xlsfinfo(xlsfile);      % get information returned by XLSINFO on the workbook
Excel      = actxserver('Excel.Application'); % open Excel as a COM Automation server
set(Excel,'Visible',0);                       % make the application invisible
 % or ExcelApp.Visible = 0;
set(Excel,'DisplayAlerts',0);                 % make Excel not display alerts (e.g., sound and confirmation)
 % or Excel.Application.DisplayAlerts = false; % or 0
Workbooks  = Excel.Workbooks;                 % get a handle to Excel's Workbooks
Workbook   = Workbooks.Open(xlsfile);         % open an Excel Workbook and activate it
Sheets     = Excel.ActiveWorkBook.Sheets;     % get the sheets in the active Workbook
num_sheets = Sheets.Count;                    % number of worksheets
num_sheets_orig = num_sheets;


% Get sheets to delete
%==========================================================================
if ischar(sheets)
    sheets = {sheets};
end
if iscell(sheets)
    sheetidx = 1:length(sheets);
elseif isnumeric(sheets) && ~isempty(sheets) && ...
        isvector(sheets)==1 && all(floor(sheets)==ceil(sheets))
    sheetidx = sort(sheets,'descend');  % it is easier to go backwards
    sheetidx(sheetidx<1) = [];          % minimum sheet index is 1
    sheetidx(sheetidx>num_sheets) = []; % maximum sheet index is num_sheets
elseif isnumeric(sheets) && isempty(sheets)
    % sheets = [];
else
    disp('Invalid input for SHEETS.')
    xls_save_and_close
    return
end


% Delete sheets
%==========================================================================
try

if ~isempty(sheets)
     
    % Delete selected worksheets
    %----------------------------
    for ss=sheetidx
        
        if num_sheets>1 % there must be at least 1 sheet
            
            if iscell(sheets)
                invoke(get(Sheets,'Item',sheets{ss}),'Delete')
            else
                invoke(get(Sheets,'Item',ss),'Delete')
                %Sheets.Item(ss).Delete;
            end
            num_sheets = num_sheets - 1;
            
        else
            if iscell(sheets)
                tmp = sheets{ss};
                if ~strcmp(tmp,Sheets.Item(1).Name) % sheet does not exist
                    break
                end
            else
                tmp = Sheets.Item(1).Name;
            end
            fprintf('%s will not be deleted. There must be at least 1 worksheet.\n',tmp)
            break
        end
        
    end
    
else
    
    % Delete the empty worksheets
    %----------------------------
    
    % Loop over sheets
    for ss=num_sheets:-1:1 % it is easier to go backwards
        %if Excel.WorksheetFunction.CountA(Sheets.Item(ss).Cells)==0
        % count the number of non-empty cells
        % COUNTA applies to Excel 2013, 2011 for Mac, 2010, 2007, 2003, XP, 2000
        if Sheets.Item(ss).UsedRange.Count == 1 && ...
                strcmp(Sheets.Item(ss).UsedRange.Rows.Address,'$A$1') && ...
                isnan(Sheets.Item(ss).Range('A1').Value)
        % If the sheet is empty or contains 1 non-empty cell,
        % UsedRange.Count=1. So the three conditions above check if the
        % sheet is really empty (there is nothing even in the first cell).
            if num_sheets>1
                Sheets.Item(ss).Delete;
                num_sheets = num_sheets - 1;
            else
                fprintf('%s will not be deleted. There must be at least 1 worksheet.\n',Sheets.Item(1).Name)
            end
        end
    end
    
end

catch ME
    disp(ME.message)
end

xls_save_and_close

if num_sheets_orig>num_sheets
    if num_sheets_orig-num_sheets>1
        fprintf('%d worksheets were deleted.\n',num_sheets_orig-num_sheets)
    else
        fprintf('1 worksheet was deleted.\n')
    end
else
    disp('Nothing was done.')
end


% Save and close
%==========================================================================
function xls_save_and_close
    Workbook.Save;   % save the workbook
    Workbooks.Close; % close the workbook
    Excel.Quit;      % quit Excel
    % or invoke(Excel,'Quit');
    delete(Excel);   % delete the handle to the ActiveX Object
    clear Excel Workbooks Workbook Sheets
end
end
function isopen = xls_check_if_open(xlsfile,action)
% 
% Determine if Excel file is open. If it is open in MS Excel, it can be
% closed.
% 
% 
%USAGE
%-----
% isopen = xls_check_if_open(xlsfile)
% isopen = xls_check_if_open(xlsfile,action)
% 
% 
%INPUT
%-----
% - XLSFILE: name of the Excel file
% - ACTION : 'close' (closes file if it is open) or '' (do nothing)
%   Option 'close' only works with MS Excel.
% 
% 
%OUTPUT
%------
% - ISOPEN:
%   1  if XLSFILE is open
%   0  if XLSFILE is not open
%   10 if XLSFILE was closed
%   11 if XLSFILE is open and could not be closed
%   -1 if an error occurred
% 
% 
% Based on "How can I determine if an XLS-file is open in Microsoft Excel,
% without using DDE commands, using MATLAB 7.7 (R2008b)?"
% (www.mathworks.com/support/solutions/en/data/1-954SDY/index.html)
% 

% Guilherme Coco Beltramini (guicoco@gmail.com)
% 2012-Dec-30, 05:21 pm

isopen = -1;

% Input
%==========================================================================

if nargin<2
    action = '';
end

if exist(xlsfile,'file')~=2
    fprintf('%s not found.\n',xlsfile)
    return
end

% The full path is required because of "Workbooks.Item(ii).FullName"
if isempty(strfind(xlsfile,filesep))
    xlsfile = fullfile(pwd,xlsfile);
end

switch action
    case ''
        close = 0;
    case 'close'
        close = 1;
    otherwise
        disp('Unknown option for ACTION.')
        return
end


% 1) Using DDE commands
%==========================================================================
% isopen = ddeinit('Excel',excelfile);
% if isopen~=0
%     isopen = 1;
% end
% But now DDEINIT has been deprecated, so ignore this option.


% 2) Using ActiveX commands
%==========================================================================
if close
    try
        
        % Check if an Excel server is running
        %------------------------------------
        Excel = actxGetRunningServer('Excel.Application');
        
        isopen = 0;
        
        Workbooks = Excel.Workbooks; % get the names of all open Excel files
        for ii = 1:Workbooks.Count
            if strcmp(xlsfile,Workbooks.Item(ii).FullName)
                isopen = 11;
                Workbooks.Item(ii).Save % save changes
                %Workbooks.Item(ii).SaveAs(filename) % save changes with a different file name
                %Workbooks.Item(ii).Saved = 1; % if you don't want to save
                Workbooks.Item(ii).Close; % close the Excel file
                isopen = 10;
                break
            end
        end
        
    catch ME
        % If Excel is not running, "actxGetRunningServer" will result in error
        if ~strcmp(ME.identifier,'MATLAB:COM:norunningserver')
            disp(ME.message)
            close = 0; % => use FOPEN
        else
            isopen = 0;
        end
    end
    
end
    
    
% 3) Using FOPEN
%==========================================================================
if ~close
    if exist(xlsfile,'file')==2 % if xlsfile does not exist, it will be created by FOPEN
        fid = fopen(xlsfile,'a');
        if fid==-1 % MATLAB is unable to open the file
            if strcmp(action,'close') % asked to close but an error occurred
                isopen = 11;
            else
                isopen = 1;
            end
        else
            isopen = 0;
            fclose(fid);
        end
    end
end
end
function p=macorwindow(p)
c=computer;
if strcmp(c(1,1:3),'MAC')==1
    p.com=0;
elseif strcmp(c(1,1:3),'PCW')==1
    p.com=1;
end

end
function [status, message]=xlwrite(filename,A,sheet, range)
% XLWRITE Write to Microsoft Excel spreadsheet file using Java
%   XLWRITE(FILE,ARRAY) writes ARRAY to the first worksheet in the Excel
%   file named FILE, starting at cell A1. It aims to have exactly the same
%   behaviour as XLSWRITE. See also XLSWRITE.
%
%   XLWRITE(FILE,ARRAY,SHEET) writes to the specified worksheet.
%
%   XLWRITE(FILE,ARRAY,RANGE) writes to the rectangular region
%   specified by RANGE in the first worksheet of the file. Specify RANGE
%   using the syntax 'C1:C2', where C1 and C2 are opposing corners of the
%   region.
%
%   XLWRITE(FILE,ARRAY,SHEET,RANGE) writes to the specified SHEET and
%   RANGE.
%
%   STATUS = XLWRITE(FILE,ARRAY,SHEET,RANGE) returns the completion
%   status of the write operation: TRUE (logical 1) for success, FALSE
%   (logical 0) for failure.  Inputs SHEET and RANGE are optional.
%
%   Input Arguments:
%
%   FILE    String that specifies the file to write. If the file does not
%           exist, XLWRITE creates a file, determining the format based on
%           the specified extension. To create a file compatible with Excel
%           97-2003 software, specify an extension of '.xls'. If you do not 
%           specify an extension, XLWRITE applies '.xls'.
%   ARRAY   Two-dimensional logical, numeric or character array or, if each
%           cell contains a single element, a cell array.
%   SHEET   Worksheet to write. One of the following:
%           * String that contains the worksheet name.
%           * Positive, integer-valued scalar indicating the worksheet
%             index.
%           If SHEET does not exist, XLWRITE adds a new sheet at the end
%           of the worksheet collection. 
%   RANGE   String that specifies a rectangular portion of the worksheet to
%           read. Not case sensitive. Use Excel A1 reference style.
%           * If you specify a SHEET, RANGE can either fit the size of
%             ARRAY or specify only the first cell (such as 'D2').
%           * If you do not specify a SHEET, RANGE must include both 
%             corners and a colon character (:), even for a single cell
%             (such as 'D2:D2').
%           * If RANGE is larger than the size of ARRAY, Excel fills the
%             remainder of the region with #N/A. If RANGE is smaller than
%             the size of ARRAY, XLWRITE writes only the subset that fits
%             into RANGE to the file.
%
%   Note
%   * This function requires the POI library to be in your javapath.
%     To add the Apache POI Library execute commands: 
%     (This assumes the POI lib files are in folder 'poi_library')
%       javaaddpath('poi_library/poi-3.8-20120326.jar');
%       javaaddpath('poi_library/poi-ooxml-3.8-20120326.jar');
%       javaaddpath('poi_library/poi-ooxml-schemas-3.8-20120326.jar');
%       javaaddpath('poi_library/xmlbeans-2.3.0.jar');
%       javaaddpath('poi_library/dom4j-1.6.1.jar');
%   * Excel converts Inf values to 65535. XLWRITE converts NaN values to
%     empty cells.
%
%   EXAMPLES
%   % Write a 7-element vector to testdata.xls:
%   xlwrite('testdata.xls', [12.7, 5.02, -98, 63.9, 0, -.2, 56])
%
%   % Write mixed text and numeric data to testdata2.xls
%   % starting at cell E1 of Sheet1:
%   d = {'Time','Temperature'; 12,98; 13,99; 14,97};
%   xlwrite('testdata2.xls', d, 1, 'E1')
%
%
%   REVISIONS
%   20121004 - First version using JExcelApi
%   20121101 - Modified to use POI library instead of JExcelApi (allows to
%           generate XLSX)
%   20121127 - Fixed bug: use existing rows if present, instead of 
%           overwrite rows by default. Thanks to Dan & Jason.
%   20121204 - Fixed bug: if a numeric sheet is given & didn't exist,
%           an error was returned instead of creating the sheet. Thanks to Marianna
%   20130106 - Fixed bug: use existing cell if present, instead of
%           overwriting. This way original XLS formatting is kept & not
%           overwritten.
%   20130125 - Fixed bug & documentation. Incorrect working of NaN. Thanks Klaus
%   20130227 - Fixed bug when no sheet number given & added Stax to java
%               load. Thanks to Thierry
%
%   Copyright 2012-2013, Alec de Zegher
%==============================================================================

% Check if POI lib is loaded
if exist('org.apache.poi.ss.usermodel.WorkbookFactory', 'class') ~= 8 ...
    || exist('org.apache.poi.hssf.usermodel.HSSFWorkbook', 'class') ~= 8 ...
    || exist('org.apache.poi.xssf.usermodel.XSSFWorkbook', 'class') ~= 8
    
    error('xlWrite:poiLibsNotLoaded',...
        'The POI library is not loaded in Matlab.\nCheck that POI jar files are in Matlab Java path!');
end

% Import required POI Java Classes
import org.apache.poi.ss.usermodel.*;
import org.apache.poi.hssf.usermodel.*;
import org.apache.poi.xssf.usermodel.*;
import org.apache.poi.ss.usermodel.*;

import org.apache.poi.ss.util.*;

status=0;

% If no sheet & xlrange is defined, attribute an empty value to it
if nargin < 3; sheet = []; end
if nargin < 4; range = []; end

% Check if sheetvariable contains range data
if nargin < 4 && ~isempty(strfind(sheet,':'))
    range = sheet;
    sheet = [];
end

% check if input data is given
if isempty(A)
    error('xlwrite:EmptyInput', 'Input array is empty!');
end
% Check that input data is not bigger than 2D
if ndims(A) > 2
	error('xlwrite:InputDimension', ...
        'Dimension of input array should not be higher than two.');
end

% Set java path to same path as Matlab path
java.lang.System.setProperty('user.dir', pwd);

% Open a file
xlsFile = java.io.File(filename);

% If file does not exist create a new workbook
if xlsFile.isFile()
    % create XSSF or HSSF workbook from existing workbook
    fileIn = java.io.FileInputStream(xlsFile);
    xlsWorkbook = WorkbookFactory.create(fileIn);
else
    % Create a new workbook based on the extension. 
    [~,~,fileExt] = fileparts(filename);
    
    % Check based on extension which type to create. If no (valid)
    % extension is given, create XLSX file
    switch lower(fileExt)
        case '.xls'
            xlsWorkbook = HSSFWorkbook();
        case '.xlsx'
            xlsWorkbook = XSSFWorkbook();
        otherwise
            xlsWorkbook = XSSFWorkbook();
            
            % Also update filename with added extension
            filename = [filename '.xlsx'];
    end
end

% If sheetname given, enter data in this sheet
if ~isempty(sheet)
    if isnumeric(sheet)
        % Java uses 0-indexing, so take sheetnumer-1
        % Check if the sheet can exist 
        if xlsWorkbook.getNumberOfSheets() >= sheet && sheet >= 1
            xlsSheet = xlsWorkbook.getSheetAt(sheet-1);
        else
            % There are less number of sheets, that the requested sheet, so
            % return an empty sheet
            xlsSheet = [];
        end
    else
        xlsSheet = xlsWorkbook.getSheet(sheet);
    end
    
    % Create a new sheet if it is empty
    if isempty(xlsSheet)
        warning('xlwrite:AddSheet', 'Added specified worksheet.');
        
        % Add the sheet
        if isnumeric(sheet)
            xlsSheet = xlsWorkbook.createSheet(['Sheet ' num2str(sheet)]);
        else
            % Create a safe sheet name
            sheet = WorkbookUtil.createSafeSheetName(sheet);
            xlsSheet = xlsWorkbook.createSheet(sheet);
        end
    end
    
else
    % check number of sheets
    nSheets = xlsWorkbook.getNumberOfSheets();
    
    % If no sheets, create one
    if nSheets < 1
        xlsSheet = xlsWorkbook.createSheet('Sheet 1');
    else
        % Select the first sheet
        xlsSheet = xlsWorkbook.getSheetAt(0);
    end
end

% if range is not specified take start row & col at A1
% locations are 0 indexed
if isempty(range)
    iRowStart = 0;
    iColStart = 0;
    iRowEnd = size(A, 1)-1;
    iColEnd = size(A, 2)-1;
else
    % Split range in start & end cell
    iSeperator = strfind(range, ':');
    if isempty(iSeperator)
        % Only start was defined as range
        % Create a helper to get the row and column
        cellStart = CellReference(range);
        iRowStart = cellStart.getRow();
        iColStart = cellStart.getCol();
        % End column calculated based on size of A
        iRowEnd = iRowStart + size(A, 1)-1;
        iColEnd = iColStart + size(A, 2)-1;
    else
        % Define start & end cell
        cellStart = range(1:iSeperator-1);
        cellEnd = range(iSeperator+1:end);
        
        % Create a helper to get the row and column
        cellStart = CellReference(cellStart);
        cellEnd = CellReference(cellEnd);
        
        % Get start & end locations
        iRowStart = cellStart.getRow();
        iColStart = cellStart.getCol();
        iRowEnd = cellEnd.getRow();
        iColEnd = cellEnd.getCol();
    end
end

% Get number of elements in A (0-indexed)
nRowA = size(A, 1)-1;
nColA = size(A, 2)-1;

% If data is a cell, convert it
if ~iscell(A)
    A = num2cell(A);
end

% Iterate over all data
for iRow = iRowStart:iRowEnd
    % Fetch the row (if it exists)
    currentRow = xlsSheet.getRow(iRow); 
    if isempty(currentRow)
        % Create a new row, as it does not exist yet
        currentRow = xlsSheet.createRow(iRow);
    end
    
    % enter data for all cols
    for iCol = iColStart:iColEnd
        % Check if cell exists
        currentCell = currentRow.getCell(iCol);
        if isempty(currentCell)
            % Create a new cell, as it does not exist yet
            currentCell = currentRow.createCell(iCol);
        end
        
        % Check if we are still in array A
        if (iRow-iRowStart)<=nRowA && (iCol-iColStart)<=nColA
            % Fetch the data
            data = A{iRow-iRowStart+1, iCol-iColStart+1};
            
            if ~isempty(data)          
                % if it is a NaN value, convert it to an empty string
                if isnumeric(data) && isnan(data)
                    data = '';
                end
                
                % Write data to cell
                currentCell.setCellValue(data);
            end

        else
            % Set field to NA
            currentCell.setCellErrorValue(FormulaError.NA.getCode());
        end
    end
end

% Write & close the workbook
fileOut = java.io.FileOutputStream(filename);
xlsWorkbook.write(fileOut);
fileOut.close();

status = 1;

end
function outtxt(filename,Lob_xy,out)

eval(['fileID=fopen(''' filename '.txt'',''w'');']);
fprintf(fileID,'%6s \r\n','Lob_xy');
fprintf(fileID,'%d %d\r\n',Lob_xy');
fprintf(fileID,'%6s \r\n','');

cel=zeros(1,size(out.x_cell,2)*2);hul=zeros(1,size(out.x_cell,2)*2);
for i=1:size(out.x_cell,2)
    cel(1,2*i-1)=out.x_cell(i);cel(1,2*i)=out.y_cell(i);
    hul(1,2*i-1)=out.x_hull(i);hul(1,2*i)=out.y_hull(i);
end
d1000=zeros(1,2000);rd1000=zeros(1,2000);
for i=1:1000
    d1000(1,2*i-1)=i/1000;d1000(1,2*i)=out.distances1000(i);
    rd1000(1,2*i-1)=i/1000;rd1000(1,2*i)=out.rev_distances1000(i);
end
DTRHrx=zeros(1,size(out.rev_distancesReal,2)*2);
for i=1:size(out.rev_distancesReal,2)
    DTRHrx(1,2*i-1)=i/5;DTRHrx(1,2*i)=out.rev_distancesReal(i);
end

fprintf(fileID,'%6s %6s \r\n','x_cell','y_cell');
fprintf(fileID,'%8.4f %8.4f\r\n',cel');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s %6s \r\n','x_hull','y_hull');
fprintf(fileID,'%8.4f %8.4f\r\n',hul');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s \r\n','DTCH');
fprintf(fileID,'%8.4f %8.4f\r\n',d1000');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s \r\n','DTRH');
fprintf(fileID,'%8.4f %8.4f\r\n',rd1000');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s \r\n','DTRH real scale');
fprintf(fileID,'%8.4f %8.4f\r\n',DTRHrx');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s %6s \r\n','area','convex area');
fprintf(fileID,'%8.4f %8.4f\r\n',[out.area;out.C_area]');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s %6s \r\n','perimeter','convex perimeter');
fprintf(fileID,'%8.4f %8.4f\r\n',[out.perimeter;out.C_perimeter]');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s\r\n','mean_radius');
fprintf(fileID,'%8.4f\r\n',out.mean_radius');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s\r\n','klobe_num');
fprintf(fileID,'%8.4f\r\n',out.klobe_num');
fprintf(fileID,'%6s \r\n','');

fprintf(fileID,'%6s %6s %6s %6s\r\n','circularity','roundness','convexity','solidity');
fprintf(fileID,'%8.4f %8.4f %8.4f %8.4f\r\n',[out.compactness;out.roundness;out.convexity;out.solidity]');
fprintf(fileID,'%6s \r\n','');

fclose(fileID);

end
function y=MA(x)
moving_wid=5;

m_wid=(moving_wid-1)/2;
if isempty(x)~=1
[a,b]=size(x);
if a>=b
   x2=x(:,1)';
   n=a;
else
   x2=x(1,:);
   n=b;
end
y=zeros(1,n);


y(1:m_wid)=x(1:m_wid);y(n-m_wid+1:n)=x(n-m_wid+1:n);
for i=m_wid+1:n-m_wid
    if x(i)==0
        y(i)=x(i);
    else
    y(i)=mean(x(i-m_wid:i+m_wid));
    end
end
end
end

% --- Executes on button press in checkbox3.
%{
function checkbox3_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox3
end
%}
% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
end

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
