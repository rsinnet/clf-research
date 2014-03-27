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


        function u = computeControl(this, qx, dqx, cons, leg)
        %%
            model = this.model;
            n = model.nExtDof;
            nb = model.nBaseDof;
            u = zeros(n, 1);
            
            % Impose controlled symmetries.
            c = this.c.cs.gains;
            Ge = reshape(Gmat(qx, [], []), [n 1]);
            Ge_CS = reshape(...
                Gmat(model.groupRotate(qx, c.gamma), [], []), ...
                [n 1]);
            u = (Ge - Ge_CS);
            
            % Stabilize the torso.
            c = this.c.ts.gains;
            qta = @(q) -(q(nb) ...
                         - q(model.COORDS_QY_STA) ...
                         - q(model.COORDS_QY_STK) ...
                         - q(model.COORDS_QY_STH));
            ut = -c.kp * (qta(qx) - c.q0) - c.kd * qta(dqx);
            u(model.COORDS_QY_NSH) = u(model.COORDS_QY_NSH) + ut;
            
            % Add stance ankle spring.
            c = this.c.stas.gains;
            u(model.COORDS_QY_STA) = ...
                this.c.cs.gains.stap*u(model.COORDS_QY_STA) ...
                - c.kp * (qx(model.COORDS_QY_STA) - c.q0) ...
                - c.kd * dqx(model.COORDS_QY_STA);
            
            % Add non-stance ankle spring.
            c = this.c.nsas.gains;
            u(model.COORDS_QY_NSA) = u(model.COORDS_QY_NSA) ...
                - c.kp * (qx(model.COORDS_QY_NSA) - c.q0) ...
                - c.kd * dqx(model.COORDS_QY_NSA);
            
            % Add knee spring.
            c = this.c.nsks.gains;
            if model.getDomainIndex(cons) == model.FIRST_DOMAIN
                u(model.COORDS_QY_NSK) = u(model.COORDS_QY_NSK) ...
                    - c.kp * (qx(model.COORDS_QY_NSK) - c.q0) ...
                    - c.kd * dqx(model.COORDS_QY_NSK);
            end
            
            % Apply the heel lift controller.
            c = this.c.sthl.gains;
            blendFunction = @(q) this.c.sthl.blendFunction(q, leg);
            u(4) = u(4) + blendFunction(qx) * ...
                   (-c.kp * (qx(4) - c.q0) ...
                    - c.kd * dqx(4));
            u(model.COORDS_QY_STA) = u(model.COORDS_QY_STA) ...
                + blendFunction(qx) * ...
                (-c.kp * (qx(model.COORDS_QY_STA) - c.q0) ...
                 - c.kd * dqx(model.COORDS_QY_STA));
            
            % Base coordinates are not actuated.
            u(1:nb) = 0;
            
            % If a knee is locked, do not apply actuation.
            if cons(find(model.STK_MASK))
                u(5) = 0;
            end
            
            if cons(find(model.NSK_MASK))
                u(8) = 0;
            end
        end
    end
end
