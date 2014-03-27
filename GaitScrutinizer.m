function varargout = GaitScrutinizer(varargin)
% GAITSCRUTINIZER MATLAB code for GaitScrutinizer.fig
%      GAITSCRUTINIZER, by itself, creates a new GAITSCRUTINIZER or
%      raises the existing singleton*.
%
%      H = GAITSCRUTINIZER returns the handle to a new
%      GAITSCRUTINIZER or the handle to the existing singleton*.
%
%      GAITSCRUTINIZER('CALLBACK',hObject,eventData,handles,...)
%      calls the local function named CALLBACK in GAITSCRUTINIZER.M
%      with the given input arguments.
%
%      GAITSCRUTINIZER('Property','Value',...) creates a new
%      GAITSCRUTINIZER or raises the existing singleton*.  Starting
%      from the left, property value pairs are applied to the GUI
%      before GaitScrutinizer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property
%      application stop. All inputs are passed to
%      GaitScrutinizer_OpeningFcn via varargin.
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows
%      only one instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help
% GaitScrutinizer

% Last Modified by GUIDE v2.5 27-Jun-2013 17:23:01

% Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @GaitScrutinizer_OpeningFcn, ...
                       'gui_OutputFcn',  @GaitScrutinizer_OutputFcn, ...
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

% --- Executes just before GaitScrutinizer is made visible.
function GaitScrutinizer_OpeningFcn(hObject, eventdata, handles, ...
                                    varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to GaitScrutinizer (see VARARGIN)

    % Choose default command line output for GaitScrutinizer
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

% UIWAIT makes GaitScrutinizer wait for user response (see UIRESUME)
% uiwait(handles.gaitScrutinizerFigure);

% --- Outputs from this function are returned to the command line.
function varargout = ...
        GaitScrutinizer_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;

%% Custom functions

function setupVariablePlot(axName, tagAll, tagNow, varargin)
    gdata = guidata(gcbf);
    hs = gdata.(GS.GUIDE_HSIM);
    n = hs.model.nExtDof;
    nr = hs.model.nRobotDof;

    m = fix(length(varargin)/2);

    ax = gdata.(axName);
    cla(ax);
    hold(ax, 'on');

    distinctColors = distinguishable_colors(nr);
    set(ax, 'ColorOrder', distinctColors);
    pObj = plot(ax, 0, zeros(nr, 1));
    set(pObj, 'Tag', tagAll);
    
    pObj = plot(ax, 0, 0);
    set(pObj, 'Tag', tagNow, ...
              'Color', GS.COLORS_HELIOTROPE, ...
              'LineStyle', GS.NOW_LINESTYLE);
    hold(ax, 'off');

    for i = 1:m
        argm = {lower(varargin{2*i-1}), varargin{2*i}};
        switch argm{1}
          case 'title'
            title(ax, argm{2});
          otherwise
            set(ax, argm{1}, argm{2});
        end
    end

function initializeContent()
% Set the background figure color.

% Ensure a simulation has been conducted.
    gdata = guidata(gcbf);
    hs = gdata.(GS.GUIDE_HSIM);

    if isempty(hs.last)
        return
    end

    sol = hs.last.solution;
    
    n = hs.model.nExtDof;
    nr = hs.model.nRobotDof;
    ns = hs.model.nSpatialDim;
    nb = hs.model.nBaseDof;
    
    set(gcbf, 'Color', GS.COLORS_PERIWINKLE)

    tspan = [0 sol(hs.last.p.steps).x(end)];

    % Position variable plot objects.
    setupVariablePlot(...
        GS.AXES_POSITION, GS.POBJ_POSITION_ALL, GS.POBJ_POSITION_NOW, ...
        'Title', GS.AXES_POSITION_TITLE, ...
        'XLim', tspan);
        
    legend(gdata.(GS.AXES_POSITION), ...
           arrayfun(@(x) sprintf('q%d', x), 1:nr, ...
                    'UniformOutput', false), ...
           'Orientation', 'Vertical', ...
           'FontSize', 8);

    
    % Velocity variable plot objects.
    setupVariablePlot(...
        GS.AXES_VELOCITY, GS.POBJ_VELOCITY_ALL, GS.POBJ_VELOCITY_NOW, ...
        'Title', GS.AXES_VELOCITY_TITLE, 'XLim', tspan);

    legend(gdata.(GS.AXES_VELOCITY), ...
           arrayfun(@(x) sprintf('dq%d', x), 1:nr, ...
                     'UniformOutput', false), ...
           'Orientation', 'Vertical', ...
           'FontSize', 8);
    
    % Actuator torque plot objects.
    setupVariablePlot(...
        GS.AXES_CONTROL, GS.POBJ_CONTROL_ALL, GS.POBJ_CONTROL_NOW, ...
        'Title', GS.AXES_CONTROL_TITLE, 'XLim', tspan);

    legend(gdata.(GS.AXES_CONTROL), ...
           arrayfun(@(x) sprintf('u%d', x), 1:nr, ...
                    'UniformOutput', false), ...
           'Orientation', 'Vertical', ...
           'FontSize', 8);

    % Plot data.
    T = cat(2, sol.x);
    X = cat(2, sol.y);
    U = cat(2, sol.u);
    Q = X(1:n, :);
    dQ = X(n+1:2*n, :);
    
    Qr = Q(nb+1:n, :);
    dQr = dQ(nb+1:n, :);
    Ur = U(nb+1:n, :);
    
    h = [gdata.(GS.AXES_POSITION) ...
         gdata.(GS.AXES_VELOCITY) ...
         gdata.(GS.AXES_CONTROL)];
    
    pObj = [findobj(h(1), 'Tag', GS.POBJ_POSITION_ALL) ...
            findobj(h(2), 'Tag', GS.POBJ_VELOCITY_ALL) ...
            findobj(h(3), 'Tag', GS.POBJ_CONTROL_ALL)];
    
    for i = 1:nr
        set(pObj(nr+1-i, 1), 'XData', T, 'YData', Qr(i, :));
        set(pObj(nr+1-i, 2), 'XData', T, 'YData', dQr(i, :));
        set(pObj(nr+1-i, 3), 'XData', T, 'YData', Ur(i, :));
    end
    
    % Animation plot axes.
    ax = gdata.(GS.AXES_ANIMATION);
    cla(ax);
    hold(ax, 'on');

    % Robot plot object.
    pObj = plot(ax, 0, 0);
    axis(ax, 'equal')
    if ns == 2
        set(ax, 'XLim', GS.ANIMATION_XLIM, ...
                'YLim', GS.ANIMATION_ZLIM);
    elseif ns == 3
        view(ax, GS.VIEW_AZIMUTH, GS.VIEW_ELEVATION);
        set(ax, 'XLim', GS.ANIMATION_XLIM, ...
                'YLim', GS.ANIMATION_YLIM, ...
                'ZLim', GS.ANIMATION_ZLIM)
    else
        throw(MException('GaitScrutinizer:InvalidSpatialDimension', ...
                         ['Spatial dimension %d is not valid, must ' ...
                          'be 2 or 3. We''re not operating in ' ...
                          'Lineland here...'], ns));
    end
    set(pObj, 'Tag', GS.POBJ_ANIMATION);

    % Contact point markers.
    pObj = plot(ax, 0, 0, 'kx');
    set(pObj, 'Tag', GS.POBJ_ANIMATION_CONTACTS);

    % Active guard markers.
    pObj = plot(ax, 0, 0, 'k*');
    set(pObj, 'Tag', GS.POBJ_ANIMATION_GUARDS_STRIKE);
    pObj = plot(ax, 0, 0, 'g*');
    set(pObj, 'Tag', GS.POBJ_ANIMATION_GUARDS_LIFT);

    hold(ax, 'off');

    % Setup the state table.
    u0 = hs.last.solution(1).u(:, 1);
    if ns == 2
        rowNamesBase = {'x', 'z', 'qy'};
    elseif ns == 3
        rowNamesBase = {'x', 'y', 'z', 'qz', 'qx', 'qy'};
    end
    rowNames = cat(2, rowNamesBase, ...
                   arrayfun(@(i) ['q' int2str(i)], 1:nr, ...
                            'UniformOutput', false));

    set(gdata.(GS.GUIDE_XTABLE), ...
        'Data', reshape([hs.last.p.ic.x0; u0], [n 3]), ...
        'ColumnName', GS.STATE_COLUMN_NAMES, ...
        'RowName', rowNames, ...
        'ColumnWidth', GS.STATE_COLUMN_WIDTH);

    set(gdata.(GS.GUIDE_TIME_SCALE), ...
        'String', num2str(hs.o.timeScale));
    set(gdata.(GS.GUIDE_NUM_STEPS), ...
        'String', num2str(hs.p.steps));

    % Setup the controller selection popup menu.
    set(gdata.(GS.GUIDE_CONTROLLER_POPUPMENU), ...
        'String', hs.getControllerList());
    updateControllerPanel();
    % Setup the solver controls.
    set(gdata.(GS.GUIDE_SOLVER_NAME), 'String', func2str(hs.p.s.solver));

    % Gait changed checkbox.
    updateSolution();


function updateContent(p)
%% Updates the content on all the outputs.
% p - slider position as a number between 0 and 1.

    gdata = guidata(gcbf);

    if nargin < 1
        p = 0;
    end

    % Update the slider.
    set(findobj(gcbf, 'Tag', GS.GUIDE_TIME_SLIDER), 'Value', p);


    % Check if an hsim object is attached.
    hs = getHsimObject();
    if isempty(hs)
        return
    end

    % Ensure a simulation has been conducted.
    if ~hs.solutionValid
        return
    end

    sol = hs.last.solution;
    model = hs.model;

    % Find the point in the gait which has the time closest to that
    % specified by the slider position. E.g., p = 0.5 corresponds to
    % halfway through the simulation.
    [hybridStepIndex, integratorIndex] = getSnapshotAddress(p);

    sol0 = sol(hybridStepIndex);
    n = model.nExtDof;

    t = sol0.x;
    x = sol0.y;
    u = sol0.u;

    t0 = t(integratorIndex);
    x0 = x(:, integratorIndex);
    u0 = u(:, integratorIndex);

    q0 = x0(1:n);
    dq0 = x0(n+1:2*n);
    
    h = [gdata.(GS.AXES_POSITION) ...
         gdata.(GS.AXES_VELOCITY) ...
         gdata.(GS.AXES_CONTROL)];

    pObj = [findobj(h(1), 'Tag', GS.POBJ_POSITION_NOW);
            findobj(h(2), 'Tag', GS.POBJ_VELOCITY_NOW);
            findobj(h(3), 'Tag', GS.POBJ_CONTROL_NOW)];

    for j = 1:3
        yl = get(h(j), 'YLim');
        set(pObj(j), 'XData', [t0 t0], 'YData', yl);
    end
    
    % Update the animation plot.
    updateAnimation(q0, sol0.cons, sol0.leg, sol0.guard);

    % Update the state information table.
    set(gdata.(GS.GUIDE_XTABLE), ...
        'Data', reshape([x0; u0], [n 3]));

    drawnow;

function updateAnimation(q0, cons, leg, guard)

    gdata = guidata(gcbf);
    h = gdata.(GS.AXES_ANIMATION);
    hm = gdata.(GS.GUIDE_HSIM).model;   % robot model
    
    % Update the robot plot object.
    pObj = findobj(h, 'Tag', GS.POBJ_ANIMATION);

    T = blkdiag(eye(hm.nBaseDof), hm.transformToAbsolute);
    P = PlotPositions(T*q0, leg);    
        
    if hm.nSpatialDim == 2
        set(pObj, 'XData', P(1, :), 'YData', P(3, :));
    elseif hm.nSpatialDim == 3
        % Use absolute coordinates to reduce plotpos from 1.5MiB
        % to 62 KiB, thus allowing higher animation frame rate (and
        % saving energy).
        set(pObj, 'XData', P(1, :), 'YData', P(2, :), ...
                  'ZData', P(3, :));
    end

    % Adjust the plot if necessary
    xl = get(h, 'XLim');

    maxXPosition = max(P(1, :));
    minXPosition = min(P(1, :));

    if maxXPosition > xl(2)
        set(h, 'XLim', [xl(1) - xl(2), 0] + .5 * (xl(2) - xl(1)) + ...
               maxXPosition);
    elseif minXPosition < xl(1)
        set(h, 'XLim', [0, xl(2) - xl(1)] - .5 * (xl(2) - xl(1)) + ...
               minXPosition);
    end

    % Update the contact point markers.
    Pc = P(:, hm.PLOTPOS_INDEX(find(cons & ~guard)));
    pObj = findobj(h, 'Tag', GS.POBJ_ANIMATION_CONTACTS);
    if hm.nSpatialDim == 2
        set(pObj, 'XData', Pc(1, :), 'YData', Pc(3, :));
    elseif hm.nSpatialDim == 3
        set(pObj, 'XData', Pc(1, :), 'YData', Pc(2, :), ...
                  'ZData', Pc(3, :));
    end
        
    % Update the strike guard markers.
    Pgs = P(:, hm.PLOTPOS_INDEX(find(guard & ~cons)));
    pObj = findobj(h, 'Tag', GS.POBJ_ANIMATION_GUARDS_STRIKE);
    if hm.nSpatialDim == 2
        set(pObj, 'XData', Pgs(1, :), 'YData', Pgs(3, :));
    elseif hm.nSpatialDim == 3
        set(pObj, 'XData', Pgs(1, :), 'YData', Pgs(2, :), ...
                  'ZData', Pgs(3, :));
    end

    % Update the lift guard markers.
    Pgl = P(:, hm.PLOTPOS_INDEX(find(guard & cons)));
    pObj = findobj(h, 'Tag', GS.POBJ_ANIMATION_GUARDS_LIFT);
    if hm.nSpatialDim == 2
        set(pObj, 'XData', Pgl(1, :), 'YData', Pgl(3, :));
    elseif hm.nSpatialDim == 3
        set(pObj, 'XData', Pgl(1, :), 'YData', Pgl(2, :), ...
                  'ZData', Pgl(3, :));
    end

function [hybridStepIndex, integratorIndex] = ...
        getSnapshotAddress(p)
    % Return the step index and data index for a specified time in
    % the gait.
    gdata = guidata(gcbf);
    sol = gdata.(GS.GUIDE_HSIM).solution;

    % Get the initial and final times for the simulation.
    ti = sol(1).x(1);
    tf = sol(end).x(end);

    tspan = tf - ti;

    % Look for time t0.
    t0 = p * tspan + ti;

    % Check if the specified time is earlier than allowable.
    if t0 < sol(1).x(1)
        fprintf(2, ['Specified a time before the beginning of ' ...
                    'the simulation.']);
        t0 = ti;
    end

    % Find the closest time to that desired in the existing solution. 
    for i = 1:length(sol)
        if t0 > sol(i).x(end)
            % Desired time after this step.
            continue
        else
            % Desired time within this step.
            tdiff = abs(sol(i).x - t0);
            hybridStepIndex = i;
            integratorIndex = find(tdiff == min(tdiff), 1);
            break
        end
    end

function attachHybridSimulation(expression)
%% Attaches a hsim2d object.  This object can provide data for the
%  displays and can be used to perform simulations and other
%  analyses.
    detachHybridSimulation();
    gdata = guidata(gcbf);
    if ~isempty(expression)
        try
            gdata.(GS.GUIDE_HSIM) = evalin('base', expression);
        catch me
            msgbox(...
                ['Error: The expression you have entered is invalid. ' ...
                 'See console for details.']);
            
            msgString = getReport(me, 'extended', ...
                                      'hyperlinks', 'default');
            
            fprintf(2, ['%s: Error evaluating hsim object. MATLAB ' ...
                        'error report follows:\n\n%s\n'], ...
                    GS.APPNAME, msgString);
            % Do not attach an hsim object.
            return
        end
        
        fprintf(['Attaching user-specified hsim object to ' ...
                 'GaitScrutinizer.\n']);
    else
        gdata.(GS.GUIDE_HSIM) = hsys.hsim2d(); % new instance
        fprintf('Attaching new hsim2d object to GaitScrutinizer.\n');
    end

    guidata(gcbf, gdata);

    hs = gdata.(GS.GUIDE_HSIM);

    if isempty(hs.solution)
        fprintf(['Simulation results not found.\n'])
        runSimulation();
    end

    initializeContent();
    updateContent();

function detachHybridSimulation()
    gdata = guidata(gcbf);

    try
        % Remove the hsim object field.
        gdata = rmfield(gdata, GS.GUIDE_HSIM);
    catch me
        if strcmp(me.identifier, 'MATLAB:rmfield:InvalidFieldname');
            % No hsim object was attached.
        else
            % Some unhandled error occurred
            keyboard
        end
        return
    end
    guidata(gcbf, gdata);
    fprintf('Detaching hsim object from GaitScrutinizer.\n');

function playAnimation(pc)
    gdata = guidata(gcbf);
    hsim = gdata.(GS.GUIDE_HSIM);
    sol = hsim.last.solution;

    tsf = hsim.o.timeScale * sol(end).x(end);
    if nargin < 1
        pc0 = get(findobj(gcbf, 'Tag', GS.GUIDE_TIME_SLIDER), 'Value');
    end
    
    % If we are at the end (or very near), always start from the
    % beginning.
    if pc0 > .99
        pc0 = 0;
    end

    pc = pc0;

    tic
    while pc < 1
        updateContent(pc);
        drawnow;
        pc = pc0 + toc / tsf;
    end

    updateContent(1);
    drawnow;

function runSimulation()
    gdata = guidata(gcbf);

    hsim = getHsimObject();
    if isempty(hsim)
        return
    end

    fprintf(['Running simulation...\n']);
    hsim
    hsim.doSim();
    fprintf('Simulation complete. Computing forces and control...\n');
    hsim.computeForces();
    fprintf('Done computing forces and control.\n');

    set(gdata.(GS.GUIDE_NUM_STEPS), ...
        'String', num2str(hsim.p.steps));

    initializeContent();
    updateContent();

function hsim = getHsimObject
    gdata = guidata(gcbf);

    % Check if an hsim object is attached.
    if ~isfield(gdata, GS.GUIDE_HSIM)
        hsim = [];
        return
    end
    hsim = gdata.(GS.GUIDE_HSIM);

% Update the gaitChanged text box and possibly other things.
function updateSolution()
    gdata = guidata(gcbf);
    hsim = getHsimObject();
    if isempty(hsim)
        return
    end

    set(gdata.(GS.GUIDE_GAITCHANGED_CHECKBOX), ...
        'Value', ~hsim.solutionValid, ...
        'Enable', 'inactive');

function updateControllerPanel()
    gdata = guidata(gcbf);
    hs = gdata.(GS.GUIDE_HSIM);
    hObject = gdata.(GS.GUIDE_CONTROLLER_POPUPMENU);

    contents = cellstr(get(hObject, 'String'));
    controllerName = contents{get(hObject, 'Value')};
    controllerLabel = hs.controllerMap(controllerName);
    controller = hs.c.(controllerLabel);

    gains = fieldnames(controller.gains);
    set(gdata.(GS.GUIDE_CONTROLLER_TABLE), ...
        'Data', cellfun(@(g) controller.gains.(g), gains), ...
        'RowName', gains);

%% Callbacks

% --- Executes on slider movement.
function timeIndexSlider_Callback(hObject, eventdata, handles)
% hObject    handle to timeIndexSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine
%        range of slider
    p = get(hObject, 'Value');
    updateContent(p);

% --- Executes during object creation, after setting all properties.
function timeIndexSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timeIndexSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end

% --- Executes during object creation, after setting all properties.
function hsimObjName_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hsimObjName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on button press in attachHsimButton.
function attachHsimButton_Callback(hObject, eventdata, handles)
% hObject    handle to attachHsimButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    hsimSymbolName = get(findobj(gcbf, 'Tag', 'hsimObjName'), 'String');
    attachHybridSimulation(hsimSymbolName);

function hsimObjName_Callback(hObject, eventdata, handles)
% hObject    handle to hsimObjName (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hsimObjName as
%        text
%        str2double(get(hObject,'String')) returns contents of
%        hsimObjName as a double

% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over attachHsimButton.
function attachHsimButton_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to attachHsimButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in rewindButton.
function rewindButton_Callback(hObject, eventdata, handles)
% hObject    handle to rewindButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    updateContent();

% --- Executes on button press in playButton.
function playButton_Callback(hObject, eventdata, handles)
% hObject    handle to playButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Start animation.
    playAnimation();

function timeScaleEditText_Callback(hObject, eventdata, handles)
% hObject    handle to timeScaleEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timeScaleEditText as text
%        str2double(get(hObject,'String')) returns contents of timeScaleEditText as a double

    timeScale = str2double(get(hObject, 'String'));
    if isnan(timeScale)
        timeScale = 1;
    end
    set(hObject, 'String', timeScale);
    hsim = handles.(GS.GUIDE_HSIM);
    hsim.o.timeScale = timeScale;

% --- Executes during object creation, after setting all properties.
function timeScaleEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timeScaleEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

% --- Executes on button press in simulateButton.
function simulateButton_Callback(hObject, eventdata, handles)
% hObject    handle to simulateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    runSimulation();

% --- Executes when entered data in editable cell(s) in xCurrentTable.
function xCurrentTable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to xCurrentTable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

function numberStepsEditText_Callback(hObject, eventdata, handles)
% hObject    handle to numberStepsEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of numberStepsEditText as text
%        str2double(get(hObject,'String')) returns contents of numberStepsEditText as a double
    nSteps = max(1, fix(str2double(get(hObject, 'String'))));
    set(hObject, 'String', num2str(nSteps));
    hsim = handles.(GS.GUIDE_HSIM);
    hsim.p.steps = nSteps;
    updateSolution();

% --- Executes during object creation, after setting all properties.
function numberStepsEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numberStepsEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
%if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%set(hObject,'BackgroundColor','white');
%end
    set(hObject,'BackgroundColor',GS.COLORS_OCHROLEUCOUS);

% --- Executes on button press in runFsolveButton.
function runFsolveButton_Callback(hObject, eventdata, handles)
% hObject    handle to runFsolveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    hs = getHsimObject();
    if isempty(hs)
        return
    end
    
    % No point running fsolve with any other solver for now.
    hs.p.s.solver = @ode45;
    set(handles.(GS.GUIDE_SOLVER_NAME), 'String', 'ode45');
    drawnow;
    
    fprintf('Running Fsolve...\n');
    hs.doFsolve();
    updateSolution();
    fprintf('Fsolve complete.\n');

% --- Executes on button press in gaitChangedCheckBox.
function gaitChangedCheckBox_Callback(hObject, eventdata, handles)
% hObject    handle to gaitChangedCheckBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of
% gaitChangedCheckBox

% --- Executes on selection change in controllerPopupMenu.
function controllerPopupMenu_Callback(hObject, eventdata, handles)
% hObject    handle to controllerPopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns
%        controllerPopupMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from
%        controllerPopupMenu
    updateControllerPanel();

% --- Executes during object creation, after setting all properties.
function controllerPopupMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to controllerPopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
%if ispc && isequal(get(hObject,'BackgroundColor'), ...
%    get(0,'defaultUicontrolBackgroundColor'))
%    set(hObject,'BackgroundColor','white');
%end
    set(hObject,'BackgroundColor',GS.COLORS_EAU_DE_NIL);

% --- Executes during object creation, after setting all properties.
function controllerTable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to controllerTable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes when entered data in editable cell(s) in controllerTable.
function controllerTable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to controllerTable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data
%	property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to
%	appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
    newGain = eventdata.NewData;

    % Check if an hsim object is attached.
    hs = getHsimObject();
    if isempty(hs) || isnan(newGain)
        return
    end

    rowNames = get(hObject, 'RowName');
    gainName = rowNames{eventdata.Indices(1)};

    controllerPopupMenu = handles.(GS.GUIDE_CONTROLLER_POPUPMENU);
    contents = cellstr(get(controllerPopupMenu, 'String'));

    controllerName = contents{get(controllerPopupMenu, 'Value')};
    controllerLabel = hs.controllerMap(controllerName);

    hs.c.(controllerLabel).gains.(gainName) = newGain;

    updateSolution();

function solverText_Callback(hObject, eventdata, handles)
% hObject    handle to solverText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of solverText as
%        text
%        str2double(get(hObject,'String')) returns contents of
%        solverText as a double

    functionName = get(hObject, 'String');
    if exist(functionName)
        hs = handles.(GS.GUIDE_HSIM);
        hs.p.s.solver = str2func(functionName);
        updateSolution();
    end

% --- Executes during object creation, after setting all properties.
function solverText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to solverText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
%if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%    set(hObject,'BackgroundColor','white');
%end
    set(hObject,'BackgroundColor',GS.COLORS_OCHROLEUCOUS);



function initialXEdit_Callback(hObject, eventdata, handles)
% hObject    handle to initialXEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initialXEdit as text
%        str2double(get(hObject,'String')) returns contents of initialXEdit as a double


% --- Executes during object creation, after setting all properties.
function initialXEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initialXEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


% --- Executes on button press in loadStateButton.
function loadStateButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadStateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check if an hsim object is attached.
    hs = getHsimObject();
    if isempty(hs)
        return
    end
    
    expression = get(handles.(GS.GUIDE_INITIALX_EDIT), 'String');
    
    try
        x0 = evalin('base', expression);
    catch me
        % Warn the user, print help to the consle and continue
        % program execution.
        msgbox(['Error: The expression you have entered is invalid. See ' ...
                'the console for details.']);
        
        msgString = getReport(me, 'extended', ...
                                  'hyperlinks', 'default');
        
        fprintf(2, ['%s: Error evaluating initial state. MATLAB ' ...
                    'error report follows:\n\n%s\n'], ...
                GS.APPNAME, msgString);
        return
    end
    n = hs.model.nExtDof;
    
    if iscell(x0)
        fprintf(2, ['%s: Initial state input must evaluate to a ' ...
                    'numeric array of size (%d, 1).\n'], ...
                GS.APPNAME, 2*n);
        return
    end
    
    if ~isequal(size(x0), [2*n, 1])
        fprintf(2, ['%s: Initial state input must evaluate to a ' ...
                    'numeric array of size (%d, 1).\n'], ...
                GS.APPNAME, 2*n);
        return
    end
    
    % Set the initial state in the attached hsim object.
    hs.p.ic.x0 = x0;
    
    updateSolution();
    


% --- Executes on button press in saveFixedPointButton.
function saveFixedPointButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveFixedPointButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check if an hsim object is attached.
    hs = getHsimObject();
    if isempty(hs)
        return
    end
    
    % FIXME I suspect this if statement will cause a bug as
    % fsolveValid may be true when it should not be due to the way
    % it is set in callbacks. I.e., there may easily be a bug where
    % the solution structure is empty but fsolveValid is
    % true. Could be bad initialization.
    if hs.fsolveValid
        hs.p.ic.x0 = hs.fsolve.xstar;
        updateSolution();
    end
    