classdef hsim2d < hsys.hsim
%%%HSIM2D Specific hsim model with feet and a torso, multidomain.
    properties
        modelClass = @hsys.hmodel2d;
    end

    methods
        function this = hsim2d()
        %% Default constructor
        %FIXME setup_toolbox
            this = this@hsys.hsim(); % call super constructor

            % Initial guess to fsolve
            xr0 = [+3.186645e-004 +2.392354e-001 -2.818133e-001 -3.087729e-003 ...
                   +5.316096e-002 -1.212324e+000 +1.233372e+000 +4.285059e-001 ...
                   +1.965329e-001]';

            %xr0 = [0.0000  0.2272  -0.2494  0.0028  0.0000 -1.2200 ...
            %       1.2925  1.9053  0.0202]';

            % Initial conditions
            this.p.ic.cons = this.model.CONS_LEG_SWAP_PRE;
            this.p.ic.eventnum = 1;
            this.p.ic.leg = -1; % left leg
            this.p.ic.x0 = this.model.get_xf_on_guard(xr0, this.p.ic.leg);
        end

        function initializeControllers(this)
            c = struct();

            % Controlled Symmetries
            c.cs = struct();
            c.cs.Name = 'Controlled Symmetries';

            c.cs.gains = struct();
            c.cs.gains.gamma = pi/60; % Simulated walking slope
            c.cs.gains.stap = .2; % Proportion of CS to apply to
                                  % stance ankle

            % Torso Stabilization Controller
            c.ts = struct();
            c.ts.Name = 'Torso Stabilizer';

            c.ts.gains = struct();
            c.ts.gains.kp = 100;
            c.ts.gains.kd = 10;
            c.ts.gains.q0 = 0;

            % Ankle Stabilization Controller
            c.stas = struct();
            c.stas.Name = 'Stance Ankle Spring';

            c.stas.gains = struct();
            c.stas.gains.kp = 30;
            c.stas.gains.kd = 1;
            c.stas.gains.q0 = -pi/5;

            c.nsas = struct();
            c.nsas.Name = 'Non-Stance Ankle Spring';

            c.nsas.gains = struct();
            c.nsas.gains.kp = 30;
            c.nsas.gains.kd = 1;
            c.nsas.gains.q0 = 0;

            c.nsks = struct(); % knee kick off
            c.nsks.Name = 'Non-Stance Knee Kick Off';

            c.nsks.gains = struct();
            c.nsks.gains.kp = 70;
            c.nsks.gains.kd = 1;
            c.nsks.gains.q0 = .5;

            % Stance Heel Lift controller
            c.sthl = struct();
            c.sthl.Name = 'Stance Heel Lift';

            c.sthl.gains = struct();
            c.sthl.gains.blendRate = 0;
            c.sthl.gains.phipx_0 = 0.05;

            % FIXME Use a sine curve for the blend parameter so it's C(1)

            c.sthl.gains.kp = 200;
            c.sthl.gains.kd = 20;
            c.sthl.gains.q0 = pi/6;
            %c.sthl.gains.q0 = @(q) pi/6 * (1);

            this.c = c;

            this.updateBlendFunctions();

            addlistener(this, 'c', 'PostSet', ...
                        @this.updateControllerCallback);
        end

        function updateControllerCallback(this, src, evnt)
            this.updateBlendFunctions();
        end

        function updateBlendFunctions(this)
            this.c.sthl.blendFunctionParameter = @(q, leg) ...
                min(1, this.c.sthl.gains.blendRate * ...
                    (this.model.phipx(q, leg) - this.c.sthl.gains.phipx_0));

            this.c.sthl.blendFunction = @(q, leg) min(...
                (this.c.sthl.blendFunctionParameter(q, leg) > 0) * ...
                this.c.sthl.blendFunctionParameter(q, leg), 1);
        end


        function ux = computeControl(this, qx, dqx, cons, leg)
        %%
        % Need to use the pinned model here.
            model = this.model;
            nx = model.nExtDof;         % extended space
            nb = model.nBaseDof;        % base coordinates
            nc = model.nRobotDof;       % robot joint configuration

            sva = this.model.svaModel.(this.model.getSvaName(leg));

            T = model.transformToAbsolute;

            % Initialize control vector.
            ux = zeros(nx, 1);

            xPi = 1:nx;                 % qx = extended space
            cPi = nb+1:nx;              % qc = robot configuration

            qc = qx(cPi);               %  joint coordinates
            dqc = dqx(cPi);

            qx0 = [zeros(nb, 1); qc];   % flat-foot extended coords
            dqx0 = [zeros(nb, 1); dqc];

            qcz = T*qc;                 % absolute sagittal coords
            dqcz = T*dqc;               % of robot configuration

            qxz = [qx(1:nb); qcz];      % abs sagittal extended
            dqxz = [dqx(1:nb); dqcz];   % coords

            qxz0 = [zeros(nb, 1); qcz];
            dqxz0 = [zeros(nb, 1); dqcz];

            %% Sagittal controllers
            % TODO Should this call the 2D version? Should each
            % controller be a class that extends a superclass? Too
            % much work at the moment.

            % Impose controlled symmetries.
            c = this.c.cs.gains;
            qxz_gamma = model.groupRotate(qxz, c.gamma);

            % The function G2Dzmat takes absolute coordinates but
            % is expressed in the relative coordinates frame, so no
            % premultiplied transformation is necessary. Also, the
            % function has the sagittal restriction applied, so the
            % first nb-1 coordinates as well as the ankle roll
            % coordinates have no effect on the computation. Other
            % functions with the z identifier are generally
            % expressed in the absolute coordinate frame.
            Gr = G2Dzmat(qxz, [], leg);
            Gr_gamma = G2Dzmat(qxz_gamma, [], leg);
            ux(cPi) = (Gr - Gr_gamma);

            % Stabilize the torso.
            c = this.c.ts.gains;
            qta = @(q) -(q(nb) ...
                         - q(model.COORDS_QY_STA) ...
                         - q(model.COORDS_QY_STK) ...
                         - q(model.COORDS_QY_STH));
            ut = -c.kp * (qta(qx) - c.q0) - c.kd * qta(dqx);
            ux(model.COORDS_QY_NSH) = ux(model.COORDS_QY_NSH) + ut;

            % Add stance ankle spring.
            c = this.c.stas.gains;
            ux(model.COORDS_QY_STA) = ...
                this.c.cs.gains.stap*ux(model.COORDS_QY_STA) ...
                - c.kp * (qx(model.COORDS_QY_STA) - c.q0) ...
                - c.kd * dqx(model.COORDS_QY_STA);

            % Add non-stance ankle spring.
            c = this.c.nsas.gains;
            ux(model.COORDS_QY_NSA) = ux(model.COORDS_QY_NSA) ...
                - c.kp * (qx(model.COORDS_QY_NSA) - c.q0) ...
                - c.kd * dqx(model.COORDS_QY_NSA);

            % Add knee spring.
            c = this.c.nsks.gains;
            if model.getDomainIndex(cons) == model.FIRST_DOMAIN
                ux(model.COORDS_QY_NSK) = ux(model.COORDS_QY_NSK) ...
                    - c.kp * (qx(model.COORDS_QY_NSK) - c.q0) ...
                    - c.kd * dqx(model.COORDS_QY_NSK);
            end

            % Apply the heel lift controller.
            c = this.c.sthl.gains;
            blendFunction = @(q) this.c.sthl.blendFunction(q, leg);
            ux(model.COORDS_QY_STA) = ux(model.COORDS_QY_STA) ...
                + blendFunction(qx) * ...
                (-c.kp * (qx(model.COORDS_QY_STA) - c.q0) ...
                 - c.kd * dqx(model.COORDS_QY_STA));

            % Base coordinates are not actuated.
            ux(1:nb) = 0;

            % If a knee is locked, do not apply actuation.
            if cons(find(model.STK_MASK))
                ux(model.COORDS_QY_STK) = 0;
            end

            if cons(find(model.NSK_MASK))
                ux(model.COORDS_QY_NSK) = 0;
            end
        end
    end
end
