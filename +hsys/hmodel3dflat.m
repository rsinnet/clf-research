%HMODEL3DFLAT   Model configuration for multidomain footed walker.
classdef hmodel3dflat < hsys.hmodelsva
    properties (Constant)
        nBaseDof = 6;
        nRobotDof = 8;
        nExtDof = 14;
        nSpatialDim = 3;

        COORDS_QX_STA = 7;
        COORDS_QY_STA = 8;
        COORDS_QY_STK = 9;
        COORDS_QY_STH = 10;
        COORDS_QY_NSH = 11;
        COORDS_QY_NSK = 12;
        COORDS_QY_NSA = 13;
        COORDS_QX_NSA = 14;

        PLOTPOS_STT = 4;
        PLOTPOS_STH = 5;
        PLOTPOS_STHIP = 14;
        PLOTPOS_NSHIP = 18;

        LAMBDA_STH = 5;
        LAMBDA_NST = 9;

        PLOTPOS_INDEX = [4 5 13 28 27 19];
    end

    properties
        transformToAbsolute;
        footLength;
        hipWidth;
    end

    methods
        function this = hmodel3dflat()
        % Default constructor
        %xdir = fileparts(mfilename('fullpath'));
            xdir = pwd;
            rmpath(fileparts(which('pnst')));
            addpath([xdir filesep 'build_old-model-3d-feet-amber-suite']);
            setup_toolbox_path(xdir);
            this = this@hsys.hmodelsva();

            % Transformation to absolute coordinates.
            ns = this.nRobotDof - 2;
            foo = arrayfun(@(x) diag(ones(ns/2+1 - x, 1), 1 - x), ...
                           1:ns/2, 'UniformOutput', false);
            foo = sum(cat(3, foo{:}), 3);
            T = blkdiag(1, [foo, zeros(ns/2); ones(ns/2), -foo], 1);
            this.transformToAbsolute = T;

            P1 = PlotPositions(zeros(this.nExtDof, 1), -1);
            this.footLength = norm(P1(:, this.PLOTPOS_STT) - ...
                                   P1(:, this.PLOTPOS_STH));
            this.hipWidth = norm(P1(:, this.PLOTPOS_STHIP) - ...
                                 P1(:, this.PLOTPOS_NSHIP));

        end

        % Overload function from hsys.hmodel to use C++
        function val = ...
                legSwapGuard(this, t, x, cons, leg, vfx);
            q = this.splitState(x);
            val = HeightNSH(q, leg); % Extract the height
        end

        % Solves for the missing coordinates necessary to place
        % the system on the guard.
        function qi = solve_qi(this, xr, leg)
            [qi,fval,exitflag,output,jacobian] = fsolve(...
                @(qi) this.guardSolve(xr, qi, leg), [-.25; 0], ...
                optimset('Display', 'none'));
        end

        function fval = guardSolve(this, xr, qi, leg)
            xf = this.x_iota(xr, qi, leg);
            [qf dqf] = this.splitState(xf);

            T = blkdiag(eye(this.nBaseDof), this.transformToAbsolute);
            qz = T * qf;

            fval = [
            % Angle of non-stance heel with ground
                hnshoz(qz, [], leg) - hnshiz(qz, [], leg);
            % Height of non-stance heel above ground
                hnshz(qz, [], leg)];
        end

        % x_iota provides an embedding from fsolve coordinates to
        % the generalized coordinate space.
        function x = x_iota(this, x, qi, leg)
        % Embed from restricted guard space to full coordinates.
            nb = this.nBaseDof;
            nr = this.nRobotDof;
            n = this.nExtDof;

            qf  = [zeros(nb,1); x(1); qi(1); 0; x(2);
                   x(3);  0;  x(4); qi(2)];
            dqf = [zeros(nb,1); x(5);  x(6); 0; x(7);
                   x(8); 0; x(9); x(10)];

            % FIXME Possible mistake here
            x = [qf; dqf];
        end

        % x_pi is the canonical projection associated with x_iota
        % and as such, like x_iota, it only applies if the system
        % is on the guard.
        function x = x_pi(this, x)
        % We basically just remove the extra coordinates which
        % should be zeros or very close.
            nx = this.nExtDof;
            nb = this.nBaseDof;

            [q, dq] = this.splitState(x);

            % We need to really make sure that x(14), the nonstance
            % ankle roll takes on a value such that the heel is
            % flat on the ground. Thus, this value should likely be
            % removed from this equation. The velocity should
            % probably still be present due to the nature of
            % three-dimensional kinematic motion. Maybe it should
            % disappear though since it might be defined under the
            % assumption that the heel edge is not rotating
            % relative to the floor.
            x = [q([this.COORDS_QX_STA;
                    this.COORDS_QY_STH; this.COORDS_QY_NSH;
                    this.COORDS_QY_NSA]);
                 dq([this.COORDS_QX_STA; this.COORDS_QY_STA;
                     this.COORDS_QY_STH; this.COORDS_QY_NSH;
                     this.COORDS_QY_NSA; this.COORDS_QX_NSA])];
        end

        function xval = phipx(this, q, leg)
        % For 3d case, need to change this to take phipx from
        % sagittal plane.
            xval = pHipX(q, [], leg);
        end

        % This functions updates the base coordinates for leg swap
        % at impact. Since the base frame does not move for this
        % model, this function is simply the identity map.
        function [qout, dqout] = ...
                swapBaseCoordinates(this, qin, dqin, leg)
            % The base frame does not move.
            qout = qin;
            dqout = dqin;

            pns = pnst(qin, [], leg);
            Rns = Rnst(qin, [], []);

            vns = vnst(qin, dqin, leg);
            wns = wnst(qin, dqin, leg);

            qy = atan2(-Rns(3,1), Rns(3,3));
            qz = atan2(-Rns(1,2), Rns(2,2));
            qx = atan2(cos(qy)*Rns(3,2), Rns(3,3));

            wx = wns(1)*cos(qz) + wns(2)*sin(qz);
            wy = sec(qz) * (wns(2)*cos(qz) - wns(1)*sin(qz));
            wz = tan(qx) * (wns(1) * sin(qz) - wns(2) * cos(qz)) + ...
                 wns(3);

            qout(1:3) = pns;
            qout(4:6) = [qz; qx; qy];

            dqout(1:3) = vns;
            dqout(4:6) = [wz; wx; wy];

        end
        
        % Overload superclass functions
        function jacobians = ...
                getJacobianStruct(this, cons, leg, legSwap)
            jacobians = getJacobianStruct@hsys.hmodelsva(...
                this, cons, leg, legSwap);
            % Remove yaw and roll constraints on appropriate foot in
            % double support.
            if sum(this.STF_MASK & cons) ...
                    && sum(this.NSF_MASK & cons)
                jacobians(2 - legSwap).activeRows(3+this.AXIS_X) = false;
                jacobians(2 - legSwap).activeRows(3+this.AXIS_Z) = false;
            end
        end
    end
end
