% setup_toolbox_path
function setup_toolbox_path(xdir)
    
    if nargin == 0
        xdir = fileparts(mfilename('fullpath'));
    end
    
    addpath([xdir filesep 'third' filesep 'spatialv2']);
    addpath([xdir filesep 'third' filesep 'strings']);
    addpath([xdir filesep 'third' filesep 'distinguishable_colors']);
    addpath([xdir filesep 'third' filesep 'yaml']);
    addpath([xdir filesep 'util']);

    snakeYaml = fullfile(xdir, 'third', 'snakeyaml', 'snakeyaml-1.10.jar');

    if ~any(strcmp(snakeYaml, javaclasspath('-dynamic')))
        % Add this by environment variable?
        javaaddpath(snakeYaml);
        fprintf('snakeyaml is setup.\n');
    end
    
    rehash;