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
