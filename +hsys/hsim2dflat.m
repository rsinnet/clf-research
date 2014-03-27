classdef hsim2dflat < hsys.hsim
%%%HSIM2DFLAT Specific hsim model with feet and a torso, multidomain.
    properties
        modelClass = @hsys.hmodel2dflat;
    end

    methods
        function this =  hsim2dflat()
        %% Default constructor
            this = this@hsys.hsim(); % call super constructor

            this.p.ic.cons = this.model.CONS_LEG_SWAP_PRE;
            this.p.ic.eventnum = 1;
            this.p.ic.leg = -1; % left leg

            % Initial guess to fsolve
            xr0 = [0.231481845812008; %sthip
                   -0.201938517208770; %nship
                   0.002528966759154;
                   -1.228671121349899; %velocities
                   1.306023338692931; %sthip
                   1.856256713237396; %nship
                   0.031434562313443];
            
            xr0 = [0.2378
                   -0.1629
                   -0.0197
                   -1.0823
                   1.2954
                   0.7088
                   0.2276];
            % [] 0 q3 q4 0 q6
            % q1 0 q3 q4 0 q6

            this.p.ic.x0 = ...
                this.model.get_xf_on_guard(xr0, this.p.ic.leg);
        end

        function initializeControllers(this)
            c = struct();

            %% Controlled symmetries
            c.cs = struct();
            c.cs.Name = 'Controlled Symmetries';

            c.cs.gains = struct();
            c.cs.gains.gamma = 0.05;   % Simulated walking slope
            c.cs.gains.stap = 1;       % Proportion of CS to apply
                                       % to stance ankle

            %% Torso stabilization controller
            c.ts = struct();
            c.ts.Name = 'Torso Stabilizer';

            c.ts.gains = struct();
            c.ts.gains.kp = 50;
            c.ts.gains.kd = 10;
            c.ts.gains.q0 = .025;

            %% Stance ankle sagittal stabilization controller
            c.stas = struct();
            c.stas.Name = 'Stance Ankle Spring';

            c.stas.gains = struct();
            c.stas.gains.kp = 30;
            c.stas.gains.kd = .1;
            c.stas.gains.q0 = -.5;

            %% Non-stance ankle sagittal stabilization controller
            c.nsas = struct();
            c.nsas.Name = 'Non-Stance Ankle Spring';

            c.nsas.gains = struct();
            c.nsas.gains.kp = 30; %5
            c.nsas.gains.kd = 3; %.1
            c.nsas.gains.q0 = 0;

            %% Non-stance scuffing prevention controller
            c.nssp = struct();
            c.nssp.Name = 'Non-Stance Scuffing Prevention';

            c.nssp.gains = struct();
            c.nssp.gains.rho = 1000;     % spatial dissipation rate
            c.nssp.gains.beta =  20;     % amplification level

            %% Knee kick-off controller
            c.nsks = struct();
            c.nsks.Name = 'Non-Stance Knee Kick Off';

            c.nsks.gains = struct();
            c.nsks.gains.kp = 70;
            c.nsks.gains.kd = 1;
            c.nsks.gains.q0 = .5;

            %% Stance heel lift controller
            %c.sthl = struct();
            %c.sthl.Name = 'Stance Heel Lift';

            %c.sthl.gains = struct();
            %c.sthl.gains.blendRate = 0;
            %c.sthl.gains.phipx_0 = 0.05;

            % FIXME Use a sine curve for the blend parameter so it's C(1)

            %c.sthl.gains.kp = 200;
            %c.sthl.gains.kd = 20;
            %c.sthl.gains.q0 = pi/6;
            %c.sthl.gains.q0 = @(q) pi/6 * (1);

            this.c = c;

            % The blend functions are lexically-scoped and create
            % closures when updated.
            this.updateBlendFunctions();

            % Add a property change listener to handle changes to
            % control parameters.
            addlistener(this, 'c', 'PostSet', ...
                        @this.updateControllerCallback);
        end

        function updateControllerCallback(this, src, evnt)
            this.updateBlendFunctions();
        end

        function updateBlendFunctions(this)
            if isfield(this.c, 'sthl')
                this.c.sthl.blendFunctionParameter = @(q, leg) ...
                    min(1, this.c.sthl.gains.blendRate * ...
                        (this.model.phipx(q, leg) - ...
                         this.c.sthl.gains.phipx_0));
                
                this.c.sthl.blendFunction = @(q, leg) min(...
                    (this.c.sthl.blendFunctionParameter(q, leg) > 0) * ...
                    this.c.sthl.blendFunctionParameter(q, leg), 1);
            end
        end

        function p = getPVector(this, leg)
        % p: [leg]
            p = leg;
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
            if isfield(this.c, 'sthl')
                c = this.c.sthl.gains;
                blendFunction = @(q) this.c.sthl.blendFunction(q, leg);
                ux(model.COORDS_QY_STA) = ux(model.COORDS_QY_STA) + ...
                    blendFunction(qx) * ...
                    (-c.kp * (qx(model.COORDS_QY_STA) - c.q0) ...
                     - c.kd * dqx(model.COORDS_QY_STA));
            end
            
            % Apply the scuffing prevention controller.
            if ~sum(cons & model.NSF_MASK)
                c = this.c.nssp.gains;
                h = max(0, HeightNST(qx, leg));  % Position error
                blend = exp(-c.rho * h);
                unsa = -blend * c.beta + ...
                        (1 - blend) * ux(model.COORDS_QY_NSA);
                ux(model.COORDS_QY_NSA) = unsa;
                % fprintf(['ep: %.5e, ev: %.5e, minH: %.5e, unsfs: ' ...
                % '%+1.5e\n'], ep, ev, minH, unsfs);
            end

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
