classdef hmodel2d < hsys.hmodel
%%%HMODEL2D
% Model configuration for multidomain footed walker.
    
%% Model configuration
%
    properties (Constant)
        nBaseDof = 3;
        nRobotDof = 6;
        nExtDof = 9;
        nSpatialDim = 2;

        COORDS_QY_STA = 4;
        COORDS_QY_STK = 5;       
        COORDS_QY_STH = 6;
        COORDS_QY_NSH = 7;
        COORDS_QY_NSK = 8;
        COORDS_QY_NSA = 9;
        
        PLOTPOS_STT = 2;
        PLOTPOS_STH = 3;
        PLOTPOS_STHIP = 6;
        PLOTPOS_NSHIP = 10;
        
        LAMBDA_STH = 3;
        LAMBDA_NST = 6;

        PLOTPOS_INDEX = [2 3 5 14 13 11];
    end
    
    properties
        transformToAbsolute;
        footLength;
        hipWidth;
    end
    
    methods        
        function this = hmodel2d()
        % Default constructor
        %xdir = fileparts(mfilename('fullpath'));
            xdir = pwd;
            rmpath(fileparts(which('pnst')));
            addpath([xdir filesep 'build-old-2d-feet']);
            setup_toolbox_path(xdir);
            this = this@hsys.hmodel();
            
            P1 = plotpos(zeros(this.nExtDof, 1), [], -1);
            this.footLength = norm(P1(:, this.PLOTPOS_STT) - ...
                                   P1(:, this.PLOTPOS_STH));
            this.hipWidth = norm(P1(:, this.PLOTPOS_STHIP) - ...
                                 P1(:, this.PLOTPOS_NSHIP));
            this.transformToAbsolute = eye(this.nRobotDof)

        end
        
        function qi = solve_qi(this, xr, leg)
            qi = fzero(@(qi) this.nonStanceHeelHeight(...
                0, this.x_iota(xr, qi, leg), [], leg, []), ...
                       -.25);
        end
        
        function x = x_iota(this, x, qi, leg)
            x = [zeros(3, 1); qi; 0; x(1:2); 0; x(3);
                 zeros(3, 1); x(4); 0; x(5:6); 0; x(7)];
            return
        end
        
        function x = x_pi(this, x, leg)
            x = x([6:7 9 this.nExtDof+[4 6:7 9]], :);
            return
        end
        
        function xval = phipx(this, q, leg)
        % For 3d case, need to change this to take phipx from
        % sagittal plane.
            hipPosition = pTb(q, [], []) - psta(q, [], []);
            xval = hipPosition(1); % Extract sagittal coordinate
        end

        function [qout, dqout] = ...
                swapBaseCoordinates(this, qin, dqin, leg)
            % The base frame does not move.
            n = this.nExtDof;
            nr = this.nRobotDof;            
            ns = this.nSpatialDim;
            
            qout = qin;
            dqout = dqin;
            
            Rq3 = [0 0 1 -ones(1,nr/2), ones(1,nr/2)];
            
            qout(1:ns) = pnst(qin, [], []);
            qout(3) = Rq3 * qin;
            
            dqout(1:ns) = vnst(qin, dqin, []);
            dqout(3) = Rq3 * dqin(1:n);
        end
    end
end
