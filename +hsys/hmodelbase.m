%HMODELBASE   This class provides an abstraction of robotic models for
%   mechanical systems with contact constraints.

% =================================================================
%> @file hmodelbase.m
%>
%> @brief Abstract definition for general dynamical system model for robots.
%>
%> @author R. W. Sinnet | http://rwsinnet.com/ | ryan@rwsinnet.com
%>
%> This class provides an abstraction of robotic models for mechanical
%> systems with contact constraints.
% ==============================================================
classdef hmodelbase < handle
%% Model configuration
%
    properties (Abstract)
        %> A cell array of spatial Jacobians for enumerated contact points.
        J;
        
        %> A cell array of spatial Jacobian time-derivatives for enumerated
        %> contact points.
        Jdot;

        cons_labels;
        nCons;

        domains;
        guards;
        dmap;
        nDefinedDomains;

        event_labels;
        el_length;

        transformToAbsolute;
        
        footLength;
        hipWidth;
    end

    properties (Constant, Abstract)
        % WTF
        STF_MASK;
        NSF_MASK;

        BadForceIndexException;
    
        nBaseDof;
        nRobotDof;
        nExtDof;
        nSpatialDim;

        % WTF
        PLOTPOS_INDEX;
end

    properties (Constant)
        BadLegValueException = MException(...
            'hmodel:getSvaName:BadLegValueException', ...
            ['Leg variable must be either -1 (left leg stance) or +1 ' ...
             '(right leg stance).']);

        NonstanceFootBelowGroundException = MException(...
            'hmodel:guard:NonstanceFootBelowGroundException', ...
            ['It appears that the nonstance foot passed through the ' ...
             'ground, most likely due to the imposed temporal ' ...
             'ordering of events.']);

    end

    methods (Abstract)
        qi = solve_qi(this, xr, leg);

        x = x_iota(this, x, qi, leg);
        xr = x_pi(this, xf, leg);

        val = phipx(this, q);

        [qout, dqout] = swapBaseCoordinates(this, qin, dqin, leg);

        Jindices = getJIndices(this, cons);
        g = getGuard(this, cons);
        eventName = getEventName(this, cons, eventIndex);
        [newCons, legSwap] = nextDomain(this, cons, eventIndex);
        fIndex = getForceIndex(this, cons, mask);
        
        %> Computes the unilateral constraint associated with leg swap.
        h_val = legSwapGuard(this, q, leg);
    end

    methods
        function setupDMap(this)
            keySet = arrayfun(@(x) {bdc(x.c)}, this.domains) ;
            valueSet = arrayfun(@(x) {x}, this.domains(:));

            this.dmap = containers.Map(keySet, valueSet);

            % number of defined domains, the possible number can be
            % determined from permutations of the constraints.
            this.nDefinedDomains = length(this.domains);
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

        function d = getDomain(this, cons)
            d = this.dmap(bdc(cons));
        end

        function n = getDomainIndex(this, cons)
            d = this.getDomain(cons);
            n = d.n;
        end

        function constraintIndex = ...
                getConstraintIndex(this, cons, eventIndex)
            d = this.getDomain(cons);
            ix = find(d.g);
            constraintIndex = ix(eventIndex);
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
    end
end
