%HMODEL2DPOINT   Model configuration for singled point-foot walker.
classdef hmodel2dpoint < hsys.hmodelsva
    properties (Constant)
        nBaseDof = 3;
        nRobotDof = 4;
        nExtDof = 7;
        nSpatialDim = 2;

        COORDS_QY_STA = 3;
        COORDS_QY_STK = 4;
        COORDS_QY_STH = 5;
        COORDS_QY_NSH = 6;
        COORDS_QY_NSK = 7;
        COORDS_QY_NSA = 3;

        PLOTPOS_STT = 1;
        PLOTPOS_STH = 1;
        PLOTPOS_STF = 1;
        PLOTPOS_NSF = 2;
        PLOTPOS_STHIP = 6;
        PLOTPOS_NSHIP = 10;

        LAMBDA_STH = 3;
        LAMBDA_NST = 5;

        PLOTPOS_INDEX = [2 3 5 14 13 11];

    end

    properties
        transformToAbsolute;
        footLength = 0;
        hipWidth = 0;
    end

    methods
        function this = hmodel2dpoint()
        % Default constructor
        %xdir = fileparts(mfilename('fullpath'));
            xdir = pwd;
            rmpath(fileparts(which('pnst')));
            addpath([xdir filesep 'build_old-model-2d-point-feet-amber-suite']);
            setup_toolbox_path(xdir);
            this = this@hsys.hmodelsva();

            % Transformation to absolute coordinates.
            ns = this.nRobotDof;

            foo = arrayfun(@(x) diag(ones(ns/2+1 - x, 1), 1 - x), ...
                           1:ns/2, 'UniformOutput', false);
            foo = sum(cat(3, foo{:}), 3);
            this.transformToAbsolute = [foo, zeros(ns/2); ones(ns/2), -foo];;

            P1 = PlotPositions(zeros(this.nExtDof, 1), -1);
        end

        % Overload function from hsys.hmodel to use C++
        function val = ...
                legSwapGuard(this, t, x, cons, leg, vfx);
            q = this.splitState(x);
            val = HeightNSF(q, leg); % Extract the height of the nonstance foot
        end

        % Solves for the missing coordinates necessary to place
        % the system on the guard.
        function qi = solve_qi(this, xr, leg)
            qi = fzero(@(qi) this.legSwapGuard(...
                0, this.x_iota(xr, qi, leg), [], leg, []), ...
                       -.25);
        end

        function fval = guardSolve(this, xr, qi, leg)
            xf = this.x_iota(xr, qi, leg);
            [qf dqf] = this.splitState(xf);

            T = blkdiag(eye(this.nBaseDof), this.transformToAbsolute);
            qz = T * qf;

            fval = hnshz(qz, [], leg);
        end

        function x = x_iota(this, x, qi, leg)
            x = [zeros(2, 1);  qi;   0;  x(1:2);  0;
                 zeros(2, 1); x(3);  0;  x(4:5);  0];
            return
        end

        function x = x_pi(this, x, leg)
            x = x([5:6 this.nExtDof+[3 5:6]], :);
            return
        end

        function xval = phipx(this, q, leg)
            xval = pHipX(q, [], leg);
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

            foo = deal(pnsf(qin, [], leg));
            qout(1:2) = foo([1 3]);
            qout(3) = Rq3 * qin;

            foo = vnsf(qin, dqin, leg);
            dqout(1:2) = foo([1 3]);
            dqout(3) = Rq3 * dqin(1:n);
        end

        function thisJacobian = ...
                initJacobian(this, bodyNumber, offset)
            % Disable 3d constraints, roll, yaw, and lateral
            % position
            thisJacobian = initJacobian@hsys.hmodelsva(...
                this, bodyNumber, offset);

            thisJacobian.activeRows(this.AXIS_Y) = false;
            thisJacobian.activeRows(3 + this.AXIS_X) = false;
            thisJacobian.activeRows(3 + this.AXIS_Z) = false;
        end
    end
end
