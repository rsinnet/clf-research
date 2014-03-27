classdef GS < handle
    properties (Constant)
        APPNAME = 'GaitScrutinizer';
        
        GUIDE_HSIM = 'hsimObj';
        GUIDE_TIME_SLIDER = 'timeIndexSlider';
        GUIDE_TIME_SCALE = 'timeScaleEditText';
        GUIDE_NUM_STEPS = 'numberStepsEditText';
        GUIDE_XTABLE = 'xCurrentTable';
        GUIDE_GAITCHANGED_CHECKBOX = 'gaitChangedCheckBox';
        GUIDE_CONTROLLER_POPUPMENU = 'controllerPopupMenu';
        GUIDE_CONTROLLER_TABLE = 'controllerTable';
        GUIDE_SOLVER_NAME = 'solverText';
        GUIDE_INITIALX_EDIT = 'initialXEdit';
        GUIDE_LOADSTATE_BUTTON = 'loadStateButton';
        
        AXES_POSITION = 'positionAxes';
        AXES_POSITION_TITLE = 'Coordinate Positions (m | rad)';
        POBJ_POSITION_ALL = 'positionAllPlotObj';
        POBJ_POSITION_NOW = 'positionNowPlotObj';
        
        AXES_VELOCITY = 'velocityAxes';
        AXES_VELOCITY_TITLE = 'Coordinate Velocities (m/s | rad/sec)';
        POBJ_VELOCITY_ALL = 'velocityAllPlotObj';
        POBJ_VELOCITY_NOW = 'velocityNowPlotObj';
        
        AXES_CONTROL = 'actuatorTorqueAxes';
        AXES_CONTROL_TITLE = 'Actuator Wrenches (N | Nm)';
        POBJ_CONTROL_ALL = 'actuatorTorqueAllPlotObj';
        POBJ_CONTROL_NOW = 'actuatorTorqueNowPlotObj';
        
        NOW_LINESTYLE = '-.';
        VIEW_AZIMUTH = -10;
        VIEW_ELEVATION = 10;
        
        AXES_ANIMATION = 'animationAxes';
        POBJ_ANIMATION = 'legsPlotObj';
        POBJ_ANIMATION_CONTACTS = 'enforcedContactsPlotObj';
        POBJ_ANIMATION_GUARDS_STRIKE = 'enforcedGuardsStrikePlotObj';
        POBJ_ANIMATION_GUARDS_LIFT = 'enforcedGuardsLiftPlotObj';
        
        ANIMATION_XLIM = [-.6 3.4];
        ANIMATION_YLIM = [-.75 .75];
        ANIMATION_ZLIM = [-.2 1.2];
        
        STATE_COLUMN_WIDTH = {'auto', 'auto', 'auto'};
        STATE_COLUMN_NAMES = {'q', 'dq', 'u'};
        
        COLORS_SMARAGDINE = hex2dec({'50' 'C8' '75'})/255;
        COLORS_PERIWINKLE = hex2dec({'CC' 'CC' 'FF'})/255;
        COLORS_OCHROLEUCOUS = hex2dec({'F6' 'F1' 'D5'})/255;
        COLORS_HELIOTROPE = hex2dec({'DF' '73' 'FF'})/255;
        COLORS_SKOBELOFF = hex2dec({'00' '74' '74'})/255;
        COLORS_EAU_DE_NIL = hex2dec({'B9' 'D1' 'A7'})/255;
    end
end
