classdef hmodelsva < hsys.hmodel
%HMODELSVA
%Model configuration for multidomain footed walker with SVA.
    
%% Model configuration
%
    
    properties 
        svaModel;
    end
    
    properties (Constant)
        AXIS_X = 1;
        AXIS_Y = 2;
        AXIS_Z = 3;
    end
    
    methods
        function this = hmodelsva()
        %% Default constructor
            this = this@hsys.hmodel();
            this.svaModel = loadModel('sva.yaml');
        end
        
        function svaName = getSvaName(this, leg)
        % Return the proper SVA model depending on which leg is the
        % stance leg.
            if leg == -1
                svaName = 'svaLeft';
            elseif leg == 1
                svaName = 'svaRight';
            else
                throw(this.BadLegValueException);
            end
        end

        function thisJacobian = ...
                initJacobian(this, bodyNumber, offset)
            % Order of Jw is x,y,z.
            n = this.nExtDof;

            thisJacobian = struct();
            thisJacobian.bodyNumber = bodyNumber;
            thisJacobian.offset = offset;

            thisJacobian.rp = zeros(3, n);
            thisJacobian.vp = zeros(3, n);
            thisJacobian.Jv = zeros(3, n);
            thisJacobian.Jw = zeros(3, n);
            thisJacobian.Jvdot = zeros(3, n);
            thisJacobian.Jwdot = zeros(3, n);
            thisJacobian.activeRows = true(1, 6);
            
            thisJacobian.activeRows(3 + this.AXIS_Y) = false;
        end

        function jacobians = ...
                getJacobianStruct(this, cons, leg, legSwap)
            % Generate struct array of requested Jacobians. Not sure
            % what the arguments should be yet.
            sva = this.svaModel.(this.getSvaName(leg));
            ji = 0;

            if isequal(this.STT_MASK & cons, this.STT_MASK)
                ji = ji + 1;
                jacobians(ji) = ...
                    this.initJacobian(this.nBaseDof, zeros(3, 1));
            elseif isequal(this.STH_MASK & cons, this.STH_MASK)
                ji = ji + 1;
                jacobians(ji) = ...
                    this.initJacobian(this.nBaseDof, ...
                                      [-this.footLength; 0; 0]);
            end

            if isequal(this.NST_MASK & cons, this.NST_MASK)
                ji = ji + 1;
                jacobians(ji) = ...
                    this.initJacobian(this.nExtDof, ...
                                      sva.offset(end, :)');
            elseif isequal(this.NSH_MASK & cons, this.NSH_MASK)
                ji = ji + 1;
                jacobians(ji) = ...
                    this.initJacobian(this.nExtDof, ...
                                      sva.offset(end, :)' - ...
                                      [this.footLength; 0; 0]);
            end
            
            if isequal(this.STF_MASK & cons, this.STF_MASK)
                % CANONICAL Y 
                jacobians(1).activeRows(3 + this.AXIS_Y) = true;
            end
            if isequal(this.NSF_MASK & cons, this.NSF_MASK)
                jacobians(end).activeRows(3 + this.AXIS_Y) = true;
            end            

        end

        function [Jh, Jhdot] = getNumericJacobian(...
            this, qe, dqe, cons, leg, legSwap)

            nb = this.nExtDof;

            % Store the kinematics in a struct to avoid recalculation.
            kins = struct('computed', cell(nb, 1), 'r', cell(nb, 1), ...
                          'v', cell(nb, 1), 'w', cell(nb, 1), ...
                          'R', cell(nb, 1), 'Rdot', cell(nb, 1), ...
                          'S', cell(nb, 1), 'Sdot', cell(nb, 1));
            [kins.computed] = deal(false);

            % Create a struct containing information on the
            % requested Jacobians.
            jacobians = this.getJacobianStruct(cons, leg, legSwap);
            nJ = length(jacobians);
            % Compute requested Jacobians.
            for ji = 1:nJ
                thisJacobian = jacobians(ji);
                effBodyNumber = thisJacobian.bodyNumber;
                kins = ...
                    this.computeKinematics(qe, dqe, leg, kins, effBodyNumber);
                %jacobianOffset = [-this.footLength(); 0; 0];

                rp = kins(effBodyNumber).r + ...
                     kins(effBodyNumber).R * thisJacobian.offset;

                vp = kins(effBodyNumber).v + ...
                     kins(effBodyNumber).Rdot * thisJacobian.offset;

                % recurse through body coordinates
                jacobians(ji) = this.recurseJacobian(effBodyNumber, ...
                                                     leg, kins, ...
                                                     thisJacobian, rp, vp);
           end

           [Jh, Jhdot] = this.conglomerateJacobians(cons, jacobians);

       end

       function [Jh, Jhdot] = ...
                conglomerateJacobians(this, cons, jacobians)
            % This function basically stacks the Jacobians but
            % removes excessive constraints.
            nJ = length(jacobians);

            Jc = arrayfun(@(x) [
                x.Jv(find(x.activeRows(1:3)), :);
                x.Jw(find(x.activeRows(4:6)), :)], ...
                          jacobians, ...
                          'UniformOutput', false);
            Jcdot = arrayfun(@(x) [
                x.Jvdot(find(x.activeRows(1:3)), :);
                x.Jwdot(find(x.activeRows(4:6)), :)], ...
                             jacobians, ...
                             'UniformOutput', false);
            
            % Add in knee constraints, if appropriate.
            n = this.nExtDof;
            A = eye(n); % matrix of basis vectors            
            if isequal(this.STK_MASK & cons, this.STK_MASK)
                Jc{end+1} = A(this.COORDS_QY_STK, :);
                Jcdot{end+1} = zeros(1, n);
            end
            
            if isequal(this.NSK_MASK & cons, this.NSK_MASK)
                Jc{end+1} = A(this.COORDS_QY_NSK, :);
                Jcdot{end+1} = zeros(1, n);
            end
            
            Jh = cat(1, Jc{:});
            Jhdot = cat(1, Jcdot{:});
        end
        
        function thisJacobian = ...
                recurseJacobian(this, bodyNumber, leg, ...
                                kins, thisJacobian, rp, vp); 
            if bodyNumber == 0
                % We can't recurse past the base body. Shouldn't
                % really get here, in theory.
                return
            end
            
            % Recurse to compute the parent body kinematics.
            sva = this.svaModel.(this.getSvaName(leg));
            parentBodyNumber = sva.parent(bodyNumber);
            if parentBodyNumber ~= 0
                thisJacobian = this.recurseJacobian(parentBodyNumber, ...
                                                    leg, kins, ...
                                                    thisJacobian, rp, vp);
            end
            
            % Do we always do this?
            useBodyFrame = true;

            kin = kins(bodyNumber);
            
            rp_rel = rp - kin.r;
            vp_rel = vp - kin.v;
            
            if sva.isRevolute(bodyNumber)
                thisJacobian.Jv(:, bodyNumber) = ...
                    skew(kin.S) * rp_rel;
                
                thisJacobian.Jvdot(:, bodyNumber) = ...
                    skew(kin.Sdot) * rp_rel + skew(kin.S) * vp_rel;
                
                if useBodyFrame
                    thisJacobian.Jw(:, bodyNumber) = ...
                        kin.R' * kin.S;
                    
                    thisJacobian.Jwdot(:, bodyNumber) = ...
                        kin.Rdot' * kin.S + kin.R' * kin.Sdot;
                else
                    thisJacobian.Jw(:, bodyNumber) = kin.Sj;
                    thisJacobian.Jwdot(:, bodyNumber) = kin.Sdot;
                end
            else
                thisJacobian.Jv(:, bodyNumber) = kin.S;
            end
        end
        
        function kin = computeKinematics(...
            this, qe, dqe, leg, kin, bodyNumber)
            % Recursively compute joint kinematics.
            
            if kin(bodyNumber).computed || bodyNumber == 0
                % Check first if the kinematics for this
                % bodyNumber has been computed.
                return
            end
            
            % Recurse to compute the parent body kinematics.
            sva = this.svaModel.(this.getSvaName(leg));
            parentBodyNumber = sva.parent(bodyNumber);
            if parentBodyNumber ~= 0
                kin = this.computeKinematics(qe, dqe, leg, kin, ...
                                             parentBodyNumber);
            end
            
            if (parentBodyNumber == 0)
                R_lambda = eye(3);
                Rdot_lambda = zeros(3);
                w_lambda = zeros(3, 1);
                r_lambda = zeros(3, 1);
                v_lambda = zeros(3, 1);
                offset = zeros(3, 1);
            else
                R_lambda = kin(parentBodyNumber).R;
                Rdot_lambda = kin(parentBodyNumber).Rdot;
                w_lambda = kin(parentBodyNumber).w;
                r_lambda = kin(parentBodyNumber).r;
                v_lambda = kin(parentBodyNumber).v;
                jb_lambda = bodyNumber;
                % Offset linear dofs
                if ~sva.isRevolute(parentBodyNumber)
                    r_lambda = r_lambda + ...
                        kin(parentBodyNumber).S * qe(bodyNumber);
                    v_lambda = v_lambda + ...
                        kin(parentBodyNumber).S * dqe(bodyNumber);
                end
                %FIXME
                offset = sva.offset(parentBodyNumber, :)';
            end
            
            ax = sva.ax(bodyNumber);
            % Axis in base frame
            kin(bodyNumber).S = rotation_axis(ax, R_lambda);
            
            if ~sva.isRevolute(bodyNumber)
                Rj = R_lambda;
                wj = w_lambda;
            else
                R_rel = rotation_matrix(ax, qe(bodyNumber));
                Rj = R_lambda * R_rel;
                wj = w_lambda + kin(bodyNumber).S * dqe(bodyNumber);
            end
            
            kin(bodyNumber).r = r_lambda + R_lambda * offset;
            kin(bodyNumber).v = v_lambda + Rdot_lambda * offset;
            
            kin(bodyNumber).w = wj;
            kin(bodyNumber).R = Rj;
            kin(bodyNumber).Rdot = skew(wj) * Rj;
            kin(bodyNumber).Sdot = rotation_axis(ax, Rdot_lambda);
            kin(bodyNumber).computed = true;
        end
        
        function [J, Jdot] = getJacobian(this, q, dq, cons, leg, legSwap)
        % This function returns the Jacobian matrix for a passed
        % array of constraints. This 'cons' variable has the form
        % described in function getJacobianStruct.
            if nargin < 6
                legSwap = false;
            end

            [J, Jdot] = ...
                this.getNumericJacobian(q, dq, cons, leg, legSwap);
        end
        
        function [Me, He] = ...
                computeUnconstrainedDynamics(this, qe, dqe, leg);
            % Evaluate the natural system dynamics.
            sva = this.svaModel.(this.getSvaName(leg));
            [Me, He] = HandC(sva, qe, dqe, {});
        end

    end
end
