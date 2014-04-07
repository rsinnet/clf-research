%HMODEL   This class provides an abstraction of robotic models for
%   mechanical systems with contact constraints.

% =================================================================
%> @file hmodel.m
%>
%> @brief Abstract definition for general dynamical system model for robots.
%>
%> @author R. W. Sinnet | http://rwsinnet.com/ | ryan@rwsinnet.com
%>
%> This class provides an abstraction of robotic models for mechanical
%> systems with contact constraints.
% ==============================================================
classdef hmodel < hsys.hmodelbase
    properties
        %> A cell array of spatial Jacobians for enumerated contact points.
        J;
        %> A cell array of spatial Jacobian time-derivatives for enumerated
        %> contact points.
        Jdot;

        %> Human-readable labels for the constraits
        cons_labels;
        
        %> The number of constraints considered in the model.
        nCons = 6;

        %> A struct array containing information about the modeled domains        
        domains;
        
        %> A struct containing the guards as fields using abbreviations for
        %> guard labels.
        guards;
        
        dmap;
        nDefinedDomains;
        
        event_labels;
        el_length;
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

            this.setupDMap();
            
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
            nonStanceHeelPosition = pnsh(q, [], leg);
            val = nonStanceHeelPosition(end); % Extract the height
        end

        function val = ...
                stanceToeHeight(this, t, x, cons, leg, vfx)
            q = this.splitState(x);
            stanceToePosition = pstt(q, [], leg);
            val = stanceToePosition(end); % Extract the height
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
