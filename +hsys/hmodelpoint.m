classdef hmodelpoint < hsys.hmodelbase
%%%HMODELPOINT Model configuration for point-footed walker.

%% Model configuration
%
    properties (Abstract)
        transformToAbsolute;
    end

    properties (Constant)
        FIRST_DOMAIN = 1;
        LAST_DOMAIN = 2;

        STF_MASK = [1 0 0 0];
        STK_MASK = [0 1 0 0];
        NSF_MASK = [0 0 1 0];
        NSK_MASK = [0 0 0 1];

        CONS_KNEE_LOCK_PRE = [1 1 0 0];
        CONS_LEG_SWAP_PRE = [1 1 0 1];
        CONS_D1_INVERSE = [];

        BadForceIndexException = MException(...
            'hmodel:getForceIndex:BadForceIndex', ...
            ['The mask argument currently requires a value of either ' ...
             'hmodel.STH_MASK or hmodel.NST_MASK.']);
    end

    properties (Abstract, Constant)
        COORDS_QY_STK;
        COORDS_QY_STH;
        COORDS_QY_NSH;
        COORDS_QY_NSK;

        PLOTPOS_STF;
        PLOTPOS_STK;
        PLOTPOS_STHIP;
        PLOTPOS_NSHIP;
        PLOTPOS_NSK;
        PLOTPOS_NSF;
    end

    methods
        function this = hmodel()
        %% Default constructor
            n = this.nExtDof;

            guard = struct();
            guard.stfs = @this.stanceFootHeight;
            guard.nskl = @this.nonStanceKneeAngle;
            this.guards = guard;

            A = eye(n);

            Jstk = A(this.COORDS_QY_STK, :);
            Jnsk = A(this.COORDS_QY_NSK, :);
            Jdotstk = zeros(1,n);
            Jdotnsk = zeros(1,n);

            this.J = {
                @(q, leg) Jstf(q, [], leg);
                @(q, leg) Jstk;
                @(q, leg) Jnsf(q, [], leg);
                @(q, leg) Jnsk
                     };

            this.Jdot = {
                @(q, dq, leg) Jstf_dot(q, dq, leg);
                @(q, dq, leg) Jdotstk;
                @(q, dq, leg) Jnsf_dot(q, dq, leg);
                @(q, dq, leg) Jdotnsk
                        };

            %                     st  ns
            % (i).c =           [f k f k]
            % knee lock
            this.domains(1).c = [1 1 0 0];
            this.domains(1).g = [0 0 0 2];
            this.domains(1).n = 1;
            % The g is not technically necessary but it lets us
            % restrict which guards to look at which should reduce
            % computation time. The n is only present to simplify code.

            % non-stance heel strike
            this.domains(2).c = [1 1 0 1];
            this.domains(2).g = [0 0 1 0];
            this.domains(2).n = 2;

            this.setupDMap();

            % Constaint labels
            % Impacts are determined by this.
            this.cons_labels = {...
                'Stance Foot', 'Stance Knee', ...
                'Nonstance Foot', 'Nonstance Knee']};
        end

        function val = ...
                nonStanceKneeAngle(this, t, x, cons, leg, vfx)
            val = x(this.COORDS_QY_NSK);
        end

        function val = ...
                nonStanceFootHeight(this, t, x, cons, leg, vfx);
            q = this.splitState(x);
            nonStanceFootPosition = pnsf(q, leg);
            val = nonStanceFootPosition(end); % Extract the height
        end

        function Jindices = getJIndices(this, cons)
            Jindices = [];

            if isequal(this.STF_MASK & cons, this.STF_MASK)
                Jindices(end+1) = 1;
            end

            if isequal(this.STK_MASK & cons, this.STK_MASK)
                Jindices(end+1) = 2;
            end

            if isequal(this.NSF_MASK & cons, this.NSF_MASK)
                Jindices(end+1) = 3;
            end

            if isequal(this.NSK_MASK & cons, this.NSK_MASK)
                Jindices(end+1) = 4;
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
                if ~cons(ix(j))
                    % Impact distance constraint
                    switch ix(j)
                      case 1
                        % non-stance knee lock
                        g{j} = this.guards.nskl;
                      case 2
                        % non-stance foot strike
                        g{j} = this.guards.nsfs;
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
                contact = 'Foot';
              case 2
                contact = 'Knee';
                types = {'Lock', 'Unlock'};
            end
            type = types{2 - ~cons(constraintIndex)};

            eventName = sprintf('%s %s %s', leg, contact, type);
        end

        function [newCons, legSwap] = nextDomain(this, cons, eventIndex)
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
    end
end
