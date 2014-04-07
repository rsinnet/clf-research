classdef hsim < handle
%%%HSIM Provides generic simulation functionality for a robot.

%% Abstract properties
% These methods and properties must be implemented by a class which
% inherits this class.
    methods (Abstract)
        u = computeControl(this, qe, dqe, cons, leg);
        initializeControllers(this);
    end

    properties (Abstract)
        modelClass;
    end

    %%

    properties
        % These properties only affect appear and not simulation.
        model; % This affects the gait but should not be changed.
        solution;
        fsolve;

        last; % Solution and parameters from before change of
              % parameters
        o; % options
    end

    properties (SetObservable)
        % These propertise affect the resulting gait.
        p; % parameters
        c; % controllers

        controllerMap; % Maps human-readable controller names to
                       % MATLAB field names
        solutionValid; % when the solution becomes invalid, so too
                       % should the fsolve results.
    end

    properties
        fsolveValid;
    end

    methods
        function this =  hsim()
        %% Default constructor
            this.p = struct(); % Parameters struct for variables
                               % which affect the dynamics.
            this.o = struct(); % Options struct for data passing
                               % for variables which DO NOT affect
                               % dynamics.
            this.model = this.modelClass();

            % Flag to check if solution needs to be recomputed.
            this.solutionValid = false;
            this.fsolveValid = false;

            % Debug mode
            this.o.debug = struct();
            this.o.debug.animation = false;
            this.o.debug.c = false;
            this.o.debug.d = true;

            % Time scale for animation. Unity is real time, bigger is slower.
            this.o.timeScale = 1;

            % Colors for console display.
            this.o.pref = struct();
            this.o.pref.gc = [0 .4 0];
            this.o.pref.bc = [.8 0 0];

            % Display simulation while running fsolve.
            this.o.animateFsolve = false;

            % Initialize the integrator parameters.
            this.initializeSolver();

            % Initialize the controllers, this must be implemented
            % in the child class.
            this.initializeControllers();

            % Setup a hash map from human-readable name to MATLAB field name.
            this.initializeControllerMap();

            this.p.steps = 1; % number of domains to step through
            this.p.ic = struct();

            % Add event handlers.
            this.initializeEvents();
        end

        function controllerList = getControllerList(this)
            controllerList = structfun(@(st) {st.Name}, this.c);
        end

        function initializeControllerMap(this)
        % Setup a hashmap
            controllerLabels = fieldnames(this.c); %keys
            controllerNames = structfun(@(st) {st.Name}, this.c); %values

            % Map the names to the labels to allow easy access to
            % controller's struct by name (from, e.g., the
            % controller select popup menu in GaitScrutinizer).
            this.controllerMap = containers.Map(controllerNames, controllerLabels);
        end


        function initializeEvents(this)
            addlistener(this, 'p', 'PreSet', ...
                        @this.handleParameterChange);
            addlistener(this, 'c', 'PreSet', ...
                        @this.handleParameterChange);
            addlistener(this, 'solutionValid', 'PostSet', ...
                        @this.handleSolutionValidChange);
        end

        function handleParameterChange(this, src, evnt)
            if this.solutionValid
                this.copySimulationResults();
                this.solutionValid = false;
            end
        end

        function handleSolutionValidChange(this, src, evnt)
            if ~this.solutionValid
                this.fsolveValid = false;
            end
        end

        function copySimulationResults(this)
            this.last.solution = this.solution;
            this.last.p = this.p;
            this.last.c = this.c;
        end

        function computeForces(this)
        % FIXME Still need to compute the constraining forces...
        % THIS FUNCTION NEEDS TO BE CALLED IF doSim() has been
        % called but not yet this function. Otherwise you will get
        % an error when you try to attach it to GaitScrutinizer.

            if ~this.solutionValid()
                this.doSim();
            end

            n = this.model.nExtDof;

            sol = this.solution;

            for i = 1:this.p.steps
                s = sol(i);
                m = length(s.x);
                sol(i).dx = nan(2*n, m);
                sol(i).u = nan(n, m);
                for j = 1:length(sol(i).x);
                    [sol(i).dx(:, j), sol(i).u(:, j), F] = ...
                        this.vf(s.x(j), s.y(:, j), s.cons, s.leg);
                    if j == 1
                        sol(i).F = nan(length(F), m);
                    end
                    sol(i).F(:, j) = F;
                end
            end
            % Set the current solution and the last solution.
            this.solution = sol;
            this.last.solution = sol;

            % Make sure the solution is valid.
            this.solutionValid = true;
        end

        function s = initializeSolver(this)
            this.p.s = struct(); % continuous-time solver
            this.p.s.Tspan = .5;
            this.p.s.solver = @ode45;
            this.p.s.MaxStep = 1e-2;
            this.p.s.options = @(this, cons, leg) odeset(...
                'MaxStep', this.p.s.MaxStep, ...
                'Events', @(t, x) this.guard(t, x, cons, leg));
        end

        function saveObject(this);
        % Save the hsim object before beginning simulation.
            dateNow = javaObject('java.util.Date');
            simpleDateFormat = ...
                javaObject('java.text.SimpleDateFormat', ...
                           'yyyyMMdd''T''HHmmss''Z''');
            utcTimeZone = javaMethod('getTimeZone', ...
                                     'java.util.TimeZone', ...
                                     'GMT');
            simpleDateFormat.setTimeZone(utcTimeZone);
            utcDateString = char(simpleDateFormat.format(dateNow));

            saveDir = 'saves';

            if ~exist(saveDir);
                mkdir(saveDir);
            end

            hsFileName = fullfile('.', saveDir, ...
                                  sprintf('hsimObject_%s.mat', ...
                                          utcDateString));
            hs = this;
            save(hsFileName, 'hs');

            fprintf('Saving hsim object to file ''%s''.\n', ...
                    hsFileName);
        end

        function doSim(this)
        %% Hybrid behavior
            this.solutionValid = true;

            p = this.p;
            sol = cell(p.steps, 1);

            x = p.ic.x0;
            cons = p.ic.cons;
            leg = p.ic.leg;
            eventnum = p.ic.eventnum;

            [~, legSwap] = this.model.nextDomain(cons, eventnum);

            %> @todo Magic number for number of steps threshold
            if p.steps > 8
                this.saveObject();
            end

            if legSwap
                % Check if the system is on the leg swap guard but the
                % non-stance foot is below the ground due to numerical
                % errors.
                h = this.model.legSwapGuard([], x, cons, leg, []);
                if (-1e-10 < h) && (h < 0)
                    p.ic.x0(this.model.nSpatialDim) = ...
                        x(this.model.nSpatialDim) - h + eps;
                    x = p.ic.x0;
                end
            end

            [x cons leg] = this.model.resetMap(0, x, ...
                                               cons, eventnum, leg);

            t0 = 0;
            for i=1:p.steps
                sol{i} = this.callSolver(x, cons, leg);

                % Show an animation.x1(14) = 0
                % animateSolution(sol{i}.x, sol{i}.y, cons, leg, model, o);

                % time offset
                sol{i}.x = sol{i}.x + t0;

                % save the new t0
                t0 = sol{i}.x(end);

                if sol{i}.ie
                    % Apply the reset map and continue
                    [x cons leg] = this.model.resetMap(0, sol{i}.y(:, end), ...
                                                 cons, sol{i}.ie, leg);
                else
                    % No guard triggered in time specified.
                    fprintf(2, ['Error: No guard triggered in maximum ' ...
                                'integration time, t_max = %f s. ' ...
                                ' Step number %d.\n'], p.s.Tspan, i);
                    % Resize the sol struct.
                    sol(i+1:this.p.steps) = [];
                    sol{i}.stats = [];
                    sol{i}.idata = [];
                    sol{i}.extdata = [];
                    %

                    % Set the number of steps to the number accomplished.
                    this.p.steps = i;

                    break
                end
            end

            this.solution = cat(2, sol{:});
            this.copySimulationResults();

            this.solutionValid = true;
        end

        %% Eigenvalues

        function doFsolve(this)
        %%
        % coordinates will need to be projected down to codim-1 embedded
        % submanifold, (technically codim-9 because we assume the stance
        % foot is not moving and both knees are locked)

            if isempty(this.fsolveValid) || isempty(this.solutionValid)
                return
            end

            if this.fsolveValid && this.solutionValid
                return
            end

            % Save the object before beginning fsolve
            this.saveObject();

            x0 = this.p.ic.x0;
            leg = this.p.ic.leg;

            st = struct();

            optimizerOptions = optimset('display', 'iter', ...
                                        'MaxFunEvals', 50000, ...
                                        'MaxIter', 10000);
            [xrstar,st.fval,st.exitflag,st.output,st.jacobian] = ...
                fsolve(@(x) this.fsl(x, leg), ...
                       this.model.x_pi(x0), ...
                       optimizerOptions);

            st.xstar = this.model.get_xf_on_guard(xrstar, this.p.ic.leg);

            this.fsolve = st;

            this.fsolveValid = true;
        end

        %% FSolve functions
        function ret = fsl(this, xr, leg)
        % satisfy guard condition
            xf = this.model.get_xf_on_guard(xr, leg);

            fprintf('Start | xr = [');
            fprintf('%+.6e ', double(xr'));
            fprintf(']\n');

            xnew = this.poincareMap(xf, this.p.ic.cons, ...
                                    this.p.ic.eventnum, ...
                                    this.p.ic.leg);

            % plot the figure
            if (this.o.animateFsolve) && (nargin > 2)
                %plotPosition(xnew, o.f.an.obj);
                domain = this.model.getDomain(this.p.ic.cons);
                updateAnimation(q0, domain.c, domain.g);
                drawnow;
            end

            ret = this.model.x_pi(xnew) - xr;
            fprintf('f(x) = %+.6e\n', dot(ret, ret));
            fprintf('++Time | End of Step Time: %s\n', datestr(now, 30));
            printHorizontalRule
        end

        function x = poincareMap(this, x, cons, eventnum, leg)
        % FIXME Will the gait be symmetric or must we take two
        % steps?

            legSwap = false;
            tic
            while ~legSwap
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

                [~, legSwap] = this.model.nextDomain(cons, ...
                                                     eventnum);
            end
            tEnd = toc;
            printHorizontalRule;
            fprintf('--Step! | Computation Time: %f seconds\n', ...
                    tEnd);
            printHorizontalRule;
        end

        function step_solution = callSolver(this, x, cons, leg)
        % Performs integration of ODEs for one hybrid domain.
            p = this.p;
            d = this.model.getDomain(cons);
            % FIXME Add the domain to here and remove guard, cons.
            step_solution = struct();
            step_solution.guard = d.g;
            step_solution.cons = cons;
            step_solution.leg = leg;
            step_solution.dx = [];
            step_solution.u = [];
            step_solution.F = [];

            % Error Handling: Make sure the heel is not below the
            % guard at the start of the final domain (of a single
            % step.)
            if d.g(5) && ...
                    nonStanceHeelHeight(this.model, [], x, cons, ...
                                        leg, []) < 0 && ...
                    dot([1, 0, 0], pnst(x, [], leg)) > ...
                    dot([1, 0, 0], pstt(x, [], leg))
                ode_solution = struct();
                ode_solution.x = 0;
                ode_solution.y = x;
                ode_solution.solver = 'ode45';
                ode_solution.xe = [];
                ode_solution.ye = [];
                ode_solution.ie = [];
            else
                ode_solution = p.s.solver(...
                    @(t, x) this.vf(t, x, cons, leg), ...
                    [0 p.s.Tspan], x, p.s.options(this, cons, ...
                                                  leg));
            end

            % Concatenate structs.
            foo = [fieldnames(step_solution)' ...
                 fieldnames(ode_solution)';
                 struct2cell(step_solution)' ...
                 struct2cell(ode_solution)'];
            step_solution = struct(foo{:});

            % FIXME check the form of ie for multiple guards
            if ~isempty(step_solution.ie) && ...
                    ~isscalar(step_solution.ie)
                throw(MException('hmodel:MultipleEvents', ...
                                 ['Multiple simultaneous events ' ...
                                  'occured.']));
            end
        end

        function [dx, u, Fhc] = vf(this, t, x, cons, leg)
            model = this.model;
            n = model.nExtDof;
            nb = model.nBaseDof;

            [qe, dqe] = model.splitState(x);

            [Me, He] = ...
                model.computeUnconstrainedDynamics(qe, dqe, leg);

            % Evaluate the control law.
            u = this.computeControl(qe, dqe, cons, leg);

            % Compute the wrenches and actuator torque/constraining
            % torque distribution map.
            [Fhc, B] = model.computeWrenches(...
                qe, dqe, u, cons, leg, Me, He);

            % Construct the control field.
            g = [zeros(size(B)); Me\B]; % control field

            % Combine controls and wrenches.
            v = [u; Fhc];

            % Compute the vector field.
            dx = [dqe; -Me\He] + g * v;

            if this.o.debug.c
                %printFootData(t, Fhc, cons, this.model.cJ_labels, o);
            end
        end

        function [value isterminal direction] = ...
                guard(this, t, x, cons, leg)
            model = this.model;

            [q, dq] = model.splitState(x);
            g = model.getGuard(cons);

            % FIXME The passed function should not be a closure and
            % this should go in the model file.
            value = cellfun(@(g) g(t, x, cons, leg, @this.vf), g);
            isterminal = ones(size(value));
            direction = -ones(size(value));

            % Error checking: we have a problem if the foot is
            % under the guard...
            %cDomain = model.getDomain(cons);

            %if cDomain.g(5)
            %    if value(end) < -1e-3; % FIX MAGIC NUMBER 1 mm
            %        throw(...
            %            model.NonstanceHeelBelowGroundException())
            %   end
            %end

            % FIXME Might need a scuffing condition or an anti-scuffing
            % controller.
            %if domain == model.nDom;
            %    p_nsh = pnsh(q);
            %    if p_nsh(1) - q(1) < .2
            %        isterminal = 0;
            %    end
            %end
        end

        function printEvent(this, cons, eventIndex)
            eventName = this.model.getEventName(cons, eventIndex);
            cprintf('*blue', '%s!\n', eventName);
        end
    end
end