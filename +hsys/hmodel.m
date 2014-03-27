classdef hmodel < handle
%%%HMODEL
% Model configuration for multidomain footed walker.
    
%% Model configuration
%
    properties
        J;
        Jdot;
        
        cons_labels;
        nCons = 6;
        
        domains;
        guards;
        dmap;
        nDefinedDomains;
        
        event_labels;
        el_length;
    end
    properties (Abstract)
        transformToAbsolute;
        footLength;
        hipWidth;
    end
    
    properties (Constant)
        FIRST_DOMAIN = 1;
        LAST_DOMAIN = 6;
        STF_MASK = [1 1 0 0 0 0];
        STT_MASK = [1 0 0 0 0 0];
        STH_MASK = [0 1 0 0 0 0];
        STK_MASK = [0 0 1 0 0 0];
        
        NSF_MASK = [0 0 0 1 1 0];
        NST_MASK = [0 0 0 1 0 0];
        NSH_MASK = [0 0 0 0 1 0];
        NSK_MASK = [0 0 0 0 0 1];
        
        CONS_KNEE_LOCK_PRE = [1 1 1 0 0 0];
        CONS_LEG_SWAP_PRE = [1 0 1 0 0 1];
        CONS_D1_INVERSE = [1 0 0 0 1 1];
        
        BadLegValueException = MException(...
            'hmodel:getSvaName:BadLegValueException', ...
            ['Leg variable must be either -1 (left leg stance) or +1 ' ...
             '(right leg stance).']);
        
        BadForceIndexException = MException(...
            'hmodel:getForceIndex:BadForceIndex', ...
            ['The mask argument currently requires a value of either ' ...
             'hmodel.STH_MASK or hmodel.NST_MASK.']);
        
        NonstanceHeelBelowGroundException = MException(...
            'hmodel:guard:NonstanceHeelBelowGroundException', ...
            ['It appears that the nonstance heel passed through the ' ...
             'ground, most likely due to the imposed temporal ' ...
             'ordering of events.']);

    end
    
    properties (Abstract, Constant)
        nBaseDof;
        nRobotDof;
        nExtDof;
        nSpatialDim;

        COORDS_QY_STA;
        COORDS_QY_STK;
        COORDS_QY_STH;
        COORDS_QY_NSH;
        COORDS_QY_NSK;
        COORDS_QY_NSA;
        
        % COMPAT point foot...
        PLOTPOS_STT;
        PLOTPOS_STH;
        PLOTPOS_STHIP;
        PLOTPOS_NSHIP;
        
        LAMBDA_STH;
        LAMBDA_NST;
        
        PLOTPOS_INDEX;
    end
    
    methods (Abstract)
        qi = solve_qi(this, xr, leg);
        
        x = x_iota(this, x, qi, leg);
        xr = x_pi(this, xf, leg);

        val = phipx(this, q);
        
        [qout, dqout] = swapBaseCoordinates(this, qin, dqin, leg);
    end
    
    methods
        function this = hmodel()
        %% Default constructor
            n = this.nExtDof;
            
            guard = struct();
            guard.stts = @this.stanceToeHeight;
            guard.nstl = @this.nonStanceToeLambda;
            guard.nskl = @this.nonStanceKneeAngle;
            guard.sthl = @this.stanceHeelLambda;
            guard.nshs = @this.nonStanceHeelHeight;
            this.guards = guard;
            
            A = eye(n);
            
            Jstk = A(this.COORDS_QY_STK, :);
            Jnsk = A(this.COORDS_QY_NSK, :);
            Jdotstk = zeros(1,n);
            Jdotnsk = zeros(1,n);
            
            this.J = {
                @(q, leg) Jstf(q, [], leg);
                @(q, leg) Jstt(q, [], leg);
                @(q, leg) Jsth(q, [], leg);
                @(q, leg) Jstk;
                @(q, leg) Jnsf(q, [], leg);
                @(q, leg) Jnst(q, [], leg);
                @(q, leg) Jnsh(q, [], leg);
                @(q, leg) Jnsk
                     };
            
            this.Jdot = {
                @(q, dq, leg) Jstf_dot(q, dq, leg);
                @(q, dq, leg) Jstt_dot(q, dq, leg);
                @(q, dq, leg) Jsth_dot(q, dq, leg);
                @(q, dq, leg) Jdotstk;
                @(q, dq, leg) Jnsf_dot(q, dq, leg); 
                @(q, dq, leg) Jnst_dot(q, dq, leg);
                @(q, dq, leg) Jnsh_dot(q, dq, leg);
                @(q, dq, leg) Jdotnsk
                        };
            
            %                     st    ns
            % (i).c =           [t h k t h k]
            % stance toe strike
            this.domains(1).c = [0 1 1 1 0 0];
            this.domains(1).g = [2 0 0 0 0 0];
            this.domains(1).n = 1;
            % The g is not technically necessary but it lets us
            % restrict which guards to look at which should reduce
            % computation time. The n is only present to simplify code.
            
            % non-stance toe lift
            this.domains(2).c = [1 1 1 1 0 0];
            this.domains(2).g = [0 0 0 3 0 0];
            this.domains(2).n = 2;
            
            % stance heel lift and knee lock
            this.domains(3).c = [1 1 1 0 0 0];
            this.domains(3).g = [0 5 0 0 0 4];
            this.domains(3).n = 3;
            
            % stance heel lift and non-stance heel strike
            this.domains(4).c = [1 1 1 0 0 1];
            this.domains(4).g = [0 6 0 0 1 0];
            this.domains(4).n = 4;
            
            % knee lock
            this.domains(5).c = [1 0 1 0 0 0];
            this.domains(5).g = [0 0 0 0 0 6];
            this.domains(5).n = 5;
            
            % non-stance heel strike
            this.domains(6).c = [1 0 1 0 0 1];
            this.domains(6).g = [0 0 0 0 1 0];
            this.domains(6).n = 6;

            keySet = arrayfun(@(x) {bdc(x.c)}, this.domains) ;
            valueSet = arrayfun(@(x) {x}, this.domains(:));
            
            this.dmap = containers.Map(keySet, valueSet);
            
            % number of defined domains, the possible number can be
            % determined from permutations of the constraints.
            this.nDefinedDomains = length(this.domains);
            
            % Constaint labels
            % Impacts are determined by this.
            this.cons_labels = {...
                'Stance Toe', 'Stance Heel', 'Stance Knee', ...
                'Nonstance Toe', 'Nonstance Heel', ['Nonstance ' ...
                                'Knee']};
        end

        function val = ...
                nonStanceToeLambda(this, t, x, cons, leg, vfx)
            [~, ~, F] = vfx(t, x, cons, leg);
            fIndex = this.getForceIndex(cons, this.NST_MASK);
            val = F(fIndex);
        end

        function val = ...
                nonStanceKneeAngle(this, t, x, cons, leg, vfx)
            val = x(this.COORDS_QY_NSK);
        end

        function val = ...
                stanceHeelLambda(this, t, x, cons, leg, vfx)
            [~, ~, F] = vfx(t, x, cons, leg);
            fIndex = this.getForceIndex(cons, this.STH_MASK);
            val = F(fIndex);
        end
        
        function val = ...
                nonStanceHeelHeight(this, t, x, cons, leg, vfx);
            q = this.splitState(x);
            nonStanceHeelPosition = pnsh(q, leg);
            val = nonStanceHeelPosition(end); % Extract the height
        end

        function val = ...
                stanceToeHeight(this, t, x, cons, leg, vfx)
            q = this.splitState(x);
            stanceToePosition = pstt(q, [], leg);
            val = stanceToePosition(end); % Extract the height
        end
        
        function [J Jdot] = getJacobian(this, q, dq, cons, leg, legSwap)
        % This function returns the Jacobian matrix for a passed
        % array of constraints. This 'cons' variable has the form
        % described in function getJIndices.
            if nargin < 6
                legSwap = false;
            end
            
            Jindices = this.getJIndices(cons);
            nJ = length(Jindices);
            
            Jc = cellfun(@(i) (this.J{Jindices(i)}(q, leg)), ...
                         num2cell(1:nJ), 'UniformOutput', false);
            
            Jcdot = cellfun(@(i) (this.Jdot{Jindices(i)}(q, dq, leg)), ...
                            num2cell(1:nJ), 'UniformOutput', false);

            % Stack the Jacobians together
            J = cat(1, Jc{:});
            Jdot = cat(1, Jcdot{:});
        end
        
        function Jindices = getJIndices(this, cons)
            Jindices = [];

            if isequal(this.STF_MASK & cons, this.STF_MASK)
                Jindices(end+1) = 1;
            elseif isequal(this.STT_MASK & cons, this.STT_MASK)
                Jindices(end+1) = 2;
            elseif isequal(this.STH_MASK & cons, this.STH_MASK)
                Jindices(end+1) = 3;
            end
            
            if isequal(this.STK_MASK & cons, this.STK_MASK)
                Jindices(end+1) = 4;
            end
            
            if isequal(this.NSF_MASK & cons, this.NSF_MASK)
                Jindices(end+1) = 5;
            elseif isequal(this.NST_MASK & cons, this.NST_MASK)
                Jindices(end+1) = 6;
            elseif isequal(this.NSH_MASK & cons, this.NSH_MASK)
                Jindices(end+1) = 7;
            end
            
            if isequal(this.NSK_MASK & cons, this.NSK_MASK)
                Jindices(end+1) = 8;
            end
        end
        
        function q = groupRotate(this, q, qr)
        % Rotate the world for controlled symmetries.
        % FIXME not always nBaseDof, should have a coordinate for
        % this, e.g., footPitchCoordinateIndex.
            nq = this.nBaseDof;
            q(nq) = q(nq) + qr;
        end
        
        function d = getDomain(this, cons)
            d = this.dmap(bdc(cons));
        end

        function n = getDomainIndex(this, cons)
            d = this.getDomain(cons);
            n = d.n;
        end

        function g = getGuard(this, cons)
            d = this.getDomain(cons);
            ix = find(d.g);
            ng = length(ix);
            
            g = cell(ng, 1);
            
            for j = 1:ng
                if cons(ix(j))
                    % Reaction force constraint
                    switch ix(j)
                      case 2
                        % stance heel lift
                        g{j} = this.guards.sthl;
                      case 4
                        % non-stance toe lift
                        g{j} = this.guards.nstl;
                    end
                else
                    % Impact distance constraint
                    switch ix(j)
                      case 1
                        % stance toe strike
                        g{j} = this.guards.stts;
                      case 5
                        % non-stance heel strike
                        g{j} = this.guards.nshs;
                      case 6
                        g{j} = this.guards.nskl;
                    end
                end
                
            end
        end

        function constraintIndex = ...
                getConstraintIndex(this, cons, eventIndex)
            d = this.getDomain(cons);
            ix = find(d.g);
            constraintIndex = ix(eventIndex);
        end


        function eventName = getEventName(this, cons, eventIndex)
            nc = this.nCons;
            constraintIndex = this.getConstraintIndex(cons, eventIndex);
            
            if constraintIndex > nc/2
                leg = 'Non-stance';
            else
                leg = 'Stance';
            end
            
            types = {'Strike', 'Lift'};
            switch mod(constraintIndex-1, nc/2) + 1
              case 1
                contact = 'Toe';
              case 2
                contact = 'Heel';
              case 3
                contact = 'Knee';
                types = {'Lock', 'Unlock'};
            end
            type = types{2 - ~cons(constraintIndex)};
            
            eventName = sprintf('%s %s %s', leg, contact, type);
        end

        function [newCons, legSwap] = nextDomain(this, cons, ...
                                                 eventIndex)
            constraintIndex = this.getConstraintIndex(cons, eventIndex);
            
            % This determines when to switch the legs, which also
            % corresponds to when to unlock the stance knee.
            legSwap = (constraintIndex == 5 && ~cons(constraintIndex));
            
            if legSwap
                % Unlock the stance knee and also lift the stance 
                % heel if necessary.
                newCons = this.CONS_D1_INVERSE;
                % something else possibly
                %xor(newCons, hmodel2d.STK_MASK ...
                %    || hmodel2d.STH_MASK);
            else
                newCons = cons;
                % Toggle the constraint that was triggered.
                newCons(constraintIndex) = ~newCons(constraintIndex);
            end
        end
        
        function xf = get_xf_on_guard(this, xr, leg)
            qi = this.solve_qi(xr, leg);
            xf = this.x_iota(xr, qi, leg);

            %nshh = this.nonStanceHeelHeight(...
            %    0, xf, [], leg, []);
            
            %if nshh < 0
            % Add minimum machine precision number to place
            % heel above ground if the current value of qi
            % places the heel below the ground.
            % FIXME this might not be enough in some cases
            % since there is a lot of arithmetic going on in
            % the computation of nshh. Floating point...
            %xf(this.nSpatialDim) = -nshh + 1e-20;
            %end
        end
        
        function [qout, dqout, newcons, newleg] = ...               
                swapLegs(this, qin, dqin, cons, leg)
            % Swap the coordinates, constraints, and stance leg
            % variable.
            
            n = this.nExtDof;
            nb = this.nBaseDof;
            nc = this.nCons;
            
            [qout, dqout] = this.swapBaseCoordinates(qin, dqin, leg);
            
            % Flip the body coordinates.
            qout(nb+1:n) = flipud(qin(nb+1:n));
            dqout(nb+1:n) = flipud(dqin(nb+1:n));
            
            % Flip the constraints.
            newcons = [cons(nc/2+1:nc), cons(1:nc/2)];
            
            % Flip the stance leg variable.
            % Left is -1, right is +1.
            newleg = -leg;
        end

        function [xplus, newCons, newLeg] = ...
                resetMap(this, t, xminus, cons, eventIndex, leg)
            fprintf('--Reset | Time: %s, Event: %s!\n', ...
                    datestr(now, 30), ...
                    this.getEventName(cons, eventIndex));
            n = this.nExtDof;
            [qminus, dqminus] = this.splitState(xminus);
            
            % FIXME
            % zIndex = model.preImpactZeroIndex{domain};
            % qminus(zIndex) = 0;

            % Later, this will have to be an iterative process that
            % depends on post-impact conditions.
            [FCons, legSwap] = this.nextDomain(cons, eventIndex);
            
            % Compute Jacobian for impact.
            JR = this.getJacobian(qminus, dqminus, FCons, leg, legSwap);
            
            %Me = Mmat(qminus, [], leg);
            Me = this.computeUnconstrainedDynamics(...
                qminus, 0*dqminus, leg);
            
            PR = [Me, -JR';
                  JR, zeros(size(JR,1))] \ ...
                 [Me*dqminus; zeros(size(JR,1),1)];
            
            qplus = qminus;
            dqplus = PR(1:n);
            
            if legSwap
                [qplus, dqplus, newCons, newLeg] = ...
                    this.swapLegs(qplus, dqplus, FCons, leg);
            else
                newCons = FCons;
                newLeg = leg;
            end
            
            xplus = [qplus; dqplus];
            
            return
        end
        
        function [q, dq] = splitState(this, x)
            n = this.nExtDof;
            q = x(1:n);
            dq = x(n+1:2*n);
        end
        
        function [Me, He] = ...
                computeUnconstrainedDynamics(this, qe, dqe, leg);
            % Evaluate the natural system dynamics.
            Me = Mmat(qe, [], leg);
            Ce = Cmat(qe, dqe, leg);
            Ge = reshape(Gmat(qe, [], leg), size(qe));
            He = Ce*dqe + Ge;
        end
        
        function [Fhc, Bhc] = computeWrenches(...
            this, qx, dqx, ux, cons, leg, Mx, Hx);
            
            % Construst the Jacobian for the 'cons' and evaluate it
            % and its time derivative at the current state, x.
            nx = this.nExtDof;
            [J Jdot] = this.getJacobian(qx, dqx, cons, leg);
            
            nhc = size(J, 1);           % Number of holonomic
                                        % constraints on system
            Bhc = [eye(nx), J'];        % Actuator torque +
                                        % constraining force
                                        % distribution map
            
            % Don't apply forces along the constraints.
            vx = [ux; zeros(nhc, 1)];
            
            % Compute the constraining wrenches (GRF, knee-lock
            % force, et cetera).            
            Fhc = -J/Mx*J'\(Jdot*dqx + J/Mx*(Bhc * vx - Hx));
        end
        
        function fIndex = getForceIndex(this, cons, mask)
        % FIXME Bad function name.
            ns = this.nSpatialDim;
            fIndex = [];
            
            % For simplicity, do this on a case by case basis.
            if isequal(mask, this.STH_MASK)
                fIndex = this.LAMBDA_STH;
            elseif isequal(mask, this.NST_MASK)
                fIndex = this.LAMBDA_NST;
            else
                throw(this.BadForceIndexException);
            end
            
        end
    end
end
