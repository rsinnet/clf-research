%HSIM3DFLAT   Specific hsim model with feet and a torso, multidomain.
classdef hsim3dks < hsys.hsim3dflat
    properties
        modelClass = @hsys.hmodel3dks;
    end

    methods
        function this =  hsim3dks()
        % Default constructor
            this = this@hsys.hsim3dflat(); % call super constructor
        end

        %> Perform deadbeat update on discrete state variables.
        function deadbeatUpdate(this, qplus, dqplus, newCons, newLeg)

            model = this.model;
            nx = model.nExtDof;         % extended space
            nb = model.nBaseDof;        % base coordinates
            nc = model.nRobotDof;       % robot joint configuration
            ns = nc - 2;                % shape space dim

            xPi = 1:nx;                 % qx = extended space
            cPi = nb+1:nx;              % qc = robot configuration
            sPi = nb+2:nx-1;            % qs = robot shape space

            qc = qplus(cPi);               %  joint coordinates
            dqc = dqplus(cPi);

            if qc(1) == 0
                return
            end

            sva = this.model.svaModel.(this.model.getSvaName(newLeg));

            [Mx0, Hx0] = HandC(sva, qplus, dqplus);

            % Use the chain and product rules.
            m_ff = Mx0(nb+1, nb+1);
            M_fq = Mx0(nb+1, nb+2:nx);

            % Compute alpha to obtain HZD.
            %yz = dqc(1) + 1/m_ff * (-lambda + M_fq*dqc(2:nc));
            lambda = m_ff * dqc(1) + M_fq*dqc(2:nc);

            fprintf('last alpha = %f, new_alpha = %f\n', ...
                    this.c.frr.gains.alpha, ...
                    -lambda / qc(1));
            this.c.frr.gains.alpha = -abs(lambda / qc(1));
        end


        % Overload
        function x = poincareMap(this, x, cons, eventnum, leg)
        % FIXME Will the gait be symmetric or must we take two
        % steps?

            stepsTaken = 0;
            tic
            while stepsTaken < 2
                [x cons leg] = ...
                    this.model.resetMap(0, x, cons, eventnum, leg);

                sol_st = this.callSolver(x, cons, leg);
                eventnum = sol_st.ie;
                x = sol_st.y(:, end);

                % Make sure we hit a guard
                if isempty(eventnum)
                    x = 100*ones(size(x));
                    break
                end

                if(this.model.getDomain(cons).n == 3 && ...
                   eventnum == 2)
                    stepsTaken = stepsTaken + 1;
                end

            end
            tEnd = toc;
            printHorizontalRule;
            fprintf('--Step! | Computation Time: %f seconds\n', ...
                    tEnd);
            printHorizontalRule;
        end
    end
end
