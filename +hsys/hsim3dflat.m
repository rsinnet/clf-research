%HSIM3DFLAT   Specific hsim model with feet and a torso, multidomain.
classdef hsim3dflat < hsys.hsim
    properties
        %modelClass = @hsys.hmodel3dflat;
    end
    
    methods
        function this =  hsim3dflat()
        %% Default constructor
            this = this@hsys.hsim(); % call super constructor
            
            this.p.ic.cons = this.model.CONS_KNEE_LOCK_PRE;
            this.p.ic.eventnum = 2;
            this.p.ic.leg = -1; % left leg
            
            % Initial guess to fsolve
            xr0 = [-0.031213214516500;
                   0.231481845812008; %sthip
                   -0.201938517208770; %nship
                   0.002528966759154;
                   -0.043060020639975; %velocities
                   -1.228671121349899
                   1.306023338692931; %sthip
                   1.856256713237396; %nship
                   0.031434562313443;
                   -0.239223498769613];
            xr0 = [0
                   0.2378
                   -0.1629
                   -0.0197
                   0
                   -1.0823
                   1.2954
                   0.7088
                   0.2276
                   0];
           xr0 = [0
                  2.079957923443091e-01
                  -1.632939706960043e-01
                  -1.037305073499228e-02
                  -0
                  -9.752237043511253e-01
                  1.106623157317802e+00
                  8.460843706742386e-01
                  1.366763944844235e-01
                  0];
           
           xr0 = [0
                  -0.0655
                  0.1002
                  -0.2119
                  -0.0440
                  0
                  0
                  -0.8522
                  1.0402
                  1.2289
                  -7.5871
                  0.5770
                  0];

            % [] q1 [] 0 q4 q5 0 q7 []
            % [] q1 q2 0 q4 q5 0 q7 q8
            
            this.p.ic.x0 = ...
                this.model.get_xf_on_guard(xr0, this.p.ic.leg);

        end
        
        function initializeControllers(this)
            c = struct();
            
            %% Controlled symmetries
            c.cs = struct(); 
            c.cs.Name = 'Controlled Symmetries';
            
            c.cs.gains = struct();
            c.cs.gains.gamma = .05;     % Simulated walking slope
            c.cs.gains.stap = 1;        % Proportion of CS to apply
                                        % to stance ankle

            %% Torso stabilization controller
            c.ts = struct();
            c.ts.Name = 'Torso Stabilizer';
            
            c.ts.gains = struct();
            c.ts.gains.kp = 100;
            c.ts.gains.kd = 5;
            c.ts.gains.q0 = .025;
            
            %% Stance ankle sagittal stabilization controller
            c.stas = struct();
            c.stas.Name = 'Stance Ankle Spring';
            
            c.stas.gains = struct();
            c.stas.gains.kp = 0;
            c.stas.gains.kd = 0;
            c.stas.gains.q0 = 0;

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
            c.nssp.gains.beta = 20;      % amplification level
            
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
            
            %% Functional Routhian reduction controller
            c.frr = struct();
            c.frr.Name = 'Reduction';
            
            c.frr.gains = struct();
            c.frr.gains.alpha = 15;
            c.frr.gains.epsilon = 10;
            
            %% Non-stance foot stabilizer
            c.nsfs = struct();
            c.nsfs.Name = 'Non-Stance Foot Stabilizer';

            c.nsfs.gains = struct();
            c.nsfs.gains.kp = 30;       % proportional gain
            c.nsfs.gains.kd = 1;        % derivative gain
            c.nsfs.gains.rho = 1000;    % spatial dissipation rate
            c.nsfs.gains.beta = 20;     % amplification level %100
            c.nsfs.gains.kp2 = 30;      % normal p gain
            c.nsfs.gains.kd2 = 1;       % normal d gain
            
            this.c = c;
            
            % The blend functions are lexically-scoped and create
            % closures when updated.
            this.updateBlendFunctions();
            
            % Add a property change listener to handle changes to
            % control parameters.
            if isfield(this.c, 'sthl')
                addlistener(this, 'c', 'PostSet', ...
                            @this.updateControllerCallback);
            end
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
        % p: [leg, alpha, epsilon]
            p = [leg; this.c.frr.gains.alpha; this.c.frr.gains.epsilon];
        end
        
        function ux = computeControl(this, qx, dqx, cons, leg)
        %%
        % Need to use the pinned model here.
            model = this.model;
            nx = model.nExtDof;         % extended space
            nb = model.nBaseDof;        % base coordinates
            nc = model.nRobotDof;       % robot joint configuration
            ns = nc - 2;                % shape space dim
            
            p = this.getPVector(leg);
            sva = this.model.svaModel.(this.model.getSvaName(leg));
            
            T = model.transformToAbsolute;
            
            % Initialize control vector.
            ux = zeros(nx, 1);
            
            xPi = 1:nx;                 % qx = extended space
            cPi = nb+1:nx;              % qc = robot configuration
            sPi = nb+2:nx-1;            % qs = robot shape space
            
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
            Gr = G2Dzmat(qxz, [], p);
            Gr_gamma = G2Dzmat(qxz_gamma, [], p);
            ux(cPi) = (Gr - Gr_gamma);
            
            % Technically, we should not use the 3D vector, qx. We
            % should apply the sagittal restriction.
            
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
            %if ~sum(cons & model.NSF_MASK)
            % Only apply the controller when the non-stance
            % foot is off the ground.
            ux(model.COORDS_QY_NSA) = ux(model.COORDS_QY_NSA) ...
                - c.kp * (qx(model.COORDS_QY_NSA) - c.q0) ...
                - c.kd * dqx(model.COORDS_QY_NSA);
            %end
            
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
            
            %% Additional 3D controllers
            
            % Non-stance foot stabilizer
            % FIXME TODO Make non-stance heels land flatly on its
            % edge with exponential PD controller.
            
            if ~sum(cons & model.NSF_MASK)
                c = this.c.nsfs.gains;
                
                ep = -hnshDiffz(qxz, [], []);      % Position error
                ev = -hnshDiffDotz(qxz, dqxz, []); % Velocity error
                
                % Blend the controllers
                minH = max(0, min(hnshoz(qxz, [], leg), ...
                                  hnshiz(qxz, [], leg)));
                blendH = exp(-c.rho * minH);

                unsfs = (1-blendH)*(-c.kp2*qx(model.COORDS_QX_NSA) ...
                                    - c.kd2*dqx(model.COORDS_QX_NSA));

                % Don't apply the controller during the beginning
                % half of the gait, based on hip position relative
                % to stance ankle.
                %if pHipX(qx, [], []) > 0
                unsfs = unsfs + ...
                        c.beta*blendH*(-c.kp*ep - c.kd*ev);
                %end
                
                ux(model.COORDS_QX_NSA) = unsfs + ...
                    (1-blendH)*ux(model.COORDS_QX_NSA);
                
                % fprintf(['ep: %.5e, ev: %.5e, minH: %.5e, unsfs: ' ...
                % '%+1.5e\n'], ep, ev, minH, unsfs);
            end
            
            
            %% Functional Routhian reduction
            % FIXME Shall reduction be applied in double support,
            % only flat foot? Look at cons(1:2) and cons(4:5). Put
            % the contents of this cell inside an IF statement.
            
            if isequal(cons & model.STF_MASK, model.STF_MASK)                
                % Compute pinned model dynamics.
                % FIXME qx0 is not defined properly if we intend to
                % do FRR when the stance foot is not flat.
                [Mx0, Hx0] = HandC(sva, qx0, dqx0);
                
                M3D = Mx0(cPi, cPi);
                H3D = Hx0(cPi);
                
                % Compute shaped dynamics with absolute coordinates and
                % convert to relative coordinates with T.
                % Ma = T' * Mazmat(qxz, dqxz, p) * T;
                % Ca = T' * Cazmat(qxz, dqxz, p) * T;
                % Ga = T' * Gazmat(qxz, dqxz, p);
                % ELWa = T' * ELWazmat(qxz, dqxz, p);
                
                Ma = T' * MShaped(qxz, p(1)) * T;
                Ca = T' * CShaped(qxz, dqxz, p(1)) * T;
                Ga = T' * GShaped(qxz, p(1));
                ELWa = T' * ELWShaped(qxz, dqxz, p(1:2));
                
                % Compute energy shaping control law.
                ux(cPi) = H3D + ...
                          (M3D / Ma) * (-Ca*dqc - Ga - ELWa + ux(cPi));
                
                % Compute the wrenches and actuator torque/constraining
                % torque distribution map for the extended system.
                [Mx, Hx] = HandC(sva, qx, dqx);
                [Fhc, Bhc] = model.computeWrenches(...
                    qx, dqx, ux, cons, leg, Mx, Hx);
                
                % Combine controls and wrenches.
                v = [ux; Fhc];
                
                % Construct the control field.
                g = [zeros(size(Bhc)); Mx\Bhc];
                
                % Constrained, controlled vector field
                fc = [dqx; -Mx\Hx] + g * v;
                
                % Reduction surface stabilization
                A = eye(length(v));
                gc = g*A(:, this.model.COORDS_QX_STA);
                
                % Use the chain and product rules.
                m_ff = Mx0(nb+1, nb+1);
                M_fq = Mx0(nb+1, nb+2:nx);
                
                dM_dx = DM1DqShaped(qxz0, p(1));
                dm_ff_dq = dM_dx(1, :);
                dM_fq_dq = dM_dx(2:nc, :);
                
                % Compute Lie derivatives.
                % FIXME, these Lie derivatives must taken into
                % account the full order dynamics.
                lambda = -p(2)*qc(1);
                yz = dqc(1) + 1/m_ff * (-lambda + M_fq*dqc(2:nc));
                
                Dyz_f = p(2)/m_ff;
                Dyz_q = -1/m_ff^2 * dm_ff_dq * (-lambda + M_fq*dqc(2:nc)) ...
                        + 1/m_ff * dqc(2:nc)' * dM_fq_dq;
                Dyz_df = 1;
                Dyz_dq = 1/m_ff * M_fq;
                
                % FIXME This controller doesn't care about base
                % coordinates. That should be fine for flat foot.
                Dyz = [zeros(1, nb), Dyz_f, Dyz_q, ...
                       zeros(1, nb), Dyz_df, Dyz_dq];

                Lfyz = Dyz * fc;
                Lgyz = Dyz * gc;

                uz = 1/Lgyz*(-Lfyz - p(3)*yz);
                
                % When uz is applied to the constrained system, it
                % will drive yz to zero. How does this factor into
                % the original system.
                
                ux(this.model.COORDS_QX_STA) = ...
                    ux(this.model.COORDS_QX_STA) + uz;
            end
        end
        %overload
        function step_solution = callSolver(this, x, cons, leg)
            step_solution = callSolver@hsys.hsim(...
                this, x, cons, leg);
            
            d = this.model.getDomain(cons);
            q = this.model.splitState(x);
            if ~d.g(5) || ...
                    HeightNSH(q, leg) >= 0 || ...
                    dot([1, 0, 0], pnst(x, [], leg)) <= ...
                    dot([1, 0, 0], pstt(x, [], leg))
                
                if ~cons(5) && d.g(5) && ~isempty(step_solution.ye)
                    T = blkdiag(eye(this.model.nBaseDof), ...
                                this.model.transformToAbsolute);
                    [qz, dqz] = this.model.splitState(...
                        blkdiag(T,T)*step_solution.ye);
                    
                    fprintf('++NSH   | hDiff: %.6e, hDiffDot: %.6e\n', ...
                            hnshDiffz(qz, [], []), ...
                            hnshDiffDotz(qz, dqz, []));
                end
            end
            
        end
        function x = poincareMap(this, x, cons, eventnum, leg)
        % FIXME Will the gait be symmetric or must we take two
        % steps?
            for i = 1:2
            x = poincareMap@hsys.hsim(this, x, cons, eventnum, ...
                                      leg);
            end
        end
    end
end
