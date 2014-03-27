function model = loadModel(yamlFile)
% Get model config, output model struct with SVA models for
% both Left and Right legs as stance legId.

    model = struct();
    
    % Get sva data
    svaConfig = cell_to_matrix_scan(loadRaw(yamlFile, []));
    
    % Set variables
    model.nBaseDof = svaConfig.nBaseDof;
    model.nExtDof = length(svaConfig.extDof);
    model.nDof = model.nExtDof - model.nBaseDof;
    
    % Make simple mapping
    %model.dofNames = cell(1, model.nDof);
    %  model.dofIndexMap = struct(); % Is this needed?
    %for ii = 1:model.nDof
    %    names = model.config.dofs(ii).names;
    %    model.dofNames{ii} = names{1};
    %    for j = 1:length(names)
    %        model.dofIndexMap.(names{j}) = ii;
    %    end
    %end

    model.svaLeft = makeSva(-1, svaConfig, model); % -1 - left
    model.svaRight = makeSva(1, svaConfig, model); %  1 - right

    function [sva] = makeSva(legId, svaConfig, model)
        sva = struct();

        if legId == -1
            legSva = svaConfig.left;
            sva.Name = 'left';
        elseif legId == 1
            legSva = svaConfig.right;
            sva.Name = 'right';
        end
        
        nExtDof = model.nExtDof;
        sva.jtype = svaConfig.extDof;
        sva.NB = model.nExtDof;
        
        sva.parent = svaConfig.parentDofs;
        
        sva.Xtree = cell(1, nExtDof);
        sva.I = cell(1, nExtDof);
        
        % Extra stuff
        sva.parentOffset = zeros(nExtDof + 1, 3);
        sva.pos = zeros(nExtDof + 1, 3);
        sva.mass = zeros(nExtDof, 1);
        sva.isRevolute = false(nExtDof, 1);
        sva.ax = zeros(nExtDof, 1);

        sva.offset = legSva.offsets;

        ax = {'x', 'y', 'z'};
        sva.legSva = legSva;
        sva.svaConfig = svaConfig;
        
        for i = 1:nExtDof
            parent = sva.parent(i);
            if parent == -1
                % Use default, previous link is parent
                parent = i - 1;
                sva.parent(i) = parent;
            end
            
            if parent == 0
                offset = [0, 0, 0];
            else
                offset = legSva.offsets(parent, :);
            end
            mass = svaConfig.masses(i);
            com = legSva.comOffsets(i, :);
            inertia = zeros(3);
            sva.Xtree{i} = xlt(offset);
            sva.I{i} = mcI(mass, com, inertia);
            sva.mass(i) = mass;
            sva.parentOffset(i + 1, :) = legSva.offsets(i, :);
            sva.pos(i + 1, :) = sva.pos(i, :) + legSva.offsets(i, :);
            type = sva.jtype{i};
            isNeg = strstartswith(type, '-');
            sva.isRevolute(i) = type(1 + isNeg) == 'R';
            sva.ax(i) = find(strcmp(type(end), ax));
            if isNeg
                sva.ax(i) = -sva.ax(i);
            end
        end
    end
end