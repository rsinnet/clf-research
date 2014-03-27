 classdef hsim3d < hsys.hsim
%%%HSIM2D Specific hsim model with feet and a torso, multidomain.
    properties
        modelClass = @hsys.hmodel3d;
    end
    
    methods
        function this =  hsim3d()
        %% Default constructor
            this = this@hsys.hsim(); % call super constructor
            
            this.p.ic.cons = this.model.CONS_LEG_SWAP_PRE;
            this.p.ic.eventnum = 1;
            this.p.ic.leg = -1; % left leg
            
            % Initial guess to fsolve
            xr0 = [+0         0            [] +2.883315e-01 -3.004634e-01 ...
                   +1.376576e-03           [] ...
                   +0         0 -1.510388e+00 +1.585345e+00 +1.311591e+00 ...
                   +1.868836e-02            0]';
            %xr0 = [-5.587522e-04 -1.355653e-03 +2.884530e-01 ...
            %       -3.019766e-01 -2.109163e-03 -3.810360e-04
            %       -1.101473e-03
            % -1.537022e+00 +2.077791e+00 +1.804247e+00 +7.478574e-01 +1.089978e-02 ]';
            %xr0 = [0.0000  0 []  0.2272  -0.2494  0.0028  [] ...
            %       0.0000 0 -1.2200  1.2925  1.9053  0.0202 0]';
            xr0 = [0 0 [] +2.883315e-01 -3.004634e-01 ...
                   +1.376576e-03 [] ...
                   +9.444985e-08 0 -1.510388e+00 +1.585345e+00 +1.311591e+00 ...
                   +1.868838e-02 0];


            % qy q1 [] 0 q4 q5 0 q7 []
            % wy q1 q2 0 q4 q5 0 q7 q8

            this.p.ic.x0 = this.model.get_xf_on_guard(xr0, ...
                                                      this.p.ic.leg);
        end

        function initializeControllers(this)
            c = struct();

            %% Controlled symmetries
            c.cs = struct();
            c.cs.Name = 'Controlled Symmetries';

            c.cs.gains = struct();
            c.cs.gains.gamma = pi/60;   % Simulated walking slope
            c.cs.gains.stap = 0;        % Proportion of CS to apply
                                        % to stance ankle

            %% Torso stabilization controller
            c.ts = struct();
            c.ts.Name = 'Torso Stabilizer';
            
            c.ts.gains = struct();
            c.ts.gains.kp = 100;
            c.ts.gains.kd = 5;
            c.ts.gains.q0 = 0;
            
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
            c.nsas.gains.kp = 5;
            c.nsas.gains.kd = .1;
            c.nsas.gains.q0 = 0;
            
            %% Knee kick-off controller
            c.nsks = struct();
            c.nsks.Name = 'Non-Stance Knee Kick Off';

            c.nsks.gains = struct();
            c.nsks.gains.kp = 70;
            c.nsks.gains.kd = 1;
            c.nsks.gains.q0 = .5;
            
            %% Stance heel lift controller
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
            
            %% Functional Routhian reduction controller
            c.frr = struct();
            c.frr.Name = 'Reduction';
            
            c.frr.gains = struct();
            c.frr.gains.alpha = 10;
            c.frr.gains.epsilon = 25;
            
            %% Non-stance foot stabilizer
            c.nsfs = struct();
            c.nsfs.Name = 'Non-Stance Foot Stabilizer';

            c.nsfs.gains = struct();
            c.nsfs.gains.kp = 1;        % proportional gain
            c.nsfs.gains.kd = .05;       % derivative gain
            c.nsfs.gains.rho = 100;     % spatial dissipation rate
            c.nsfs.gains.beta = 1000;   % amplication level
            c.nsfs.gains.kp2 = 5;
            c.nsfs.gains.kd2 = .1
            
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
            this.c.sthl.blendFunctionParameter = @(q, leg) ...
                min(1, this.c.sthl.gains.blendRate * ...
                    (this.model.phipx(q, leg) - ...
                     this.c.sthl.gains.phipx_0));
            
            this.c.sthl.blendFunction = @(q, leg) min(...
                (this.c.sthl.blendFunctionParameter(q, leg) > 0) * ...
                this.c.sthl.blendFunctionParameter(q, leg), 1);
        end
        
        function p = getPVector(this, leg)
        % p: [leg, alpha, epsilon]
            p = [leg, this.c.frr.gains.alpha, this.c.frr.gains.epsilon];
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
            Gr = G2Dzmat(qxz_gamma, [], p);
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
            ux(model.COORDS_QY_STA) = ux(model.COORDS_QY_STA) + blendFunction(qx) * ...
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
            
            %% Additional 3D controllers
            
            % Non-stance foot stabilizer
            % FIXME TODO Make non-stance heels land flatly on its
            % edge with exponential PD controller.
            
            if ~sum(cons & model.NSF_MASK)
                c = this.c.nsfs.gains;
                
                ep = -hnshDiffz(qxz, [], []);      % Position error
                ev = -hnshDiffDotz(qxz, dqxz, []); % Velocity error
                
                minH = min([hnshoz(qxz, [], leg) ...
                            hnshiz(qxz, [], leg)]);
                blendH = exp(-c.rho * minH);
                unsfs = c.beta*blendH * (-c.kp*ep - c.kd*ev) ...
                        + (1 - blendH) * (-c.kp2*qx(model.COORDS_QX_NSA) ...
                           - c.kd2*dqx(model.COORDS_QX_NSA));
                
                ux(model.COORDS_QX_NSA) = ux(model.COORDS_QX_NSA) ...
                    + unsfs;
                
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
                Ma = T' * Mazmat(qxz, dqxz, p) * T;
                Ca = T' * Cazmat(qxz, dqxz, p) * T;
                Ga = T' * Gazmat(qxz, dqxz, p);
                ELWa = T' * ELWazmat(qxz, dqxz, p);
                
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
                
                dM_dx = DM1Dqmat(qx0, dqx0, p);
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
    end
end
