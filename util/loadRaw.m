function [raw] = loadRaw(configFilePath, raw, throwIfNone)

if nargin < 2
	raw = struct();
end
if nargin < 3
    throwIfNone = true;
end

% If raw is [], will return config object directly
assert(~islogical(raw));

if ~exist(configFilePath, 'file')
    if throwIfNone
        error('Config file does not exist: %s', configFilePath);
    else
        raw.config = [];
    end
else
	config = yaml_read_file(configFilePath);
	if isempty(raw)
		raw = config;
	else
		raw.config = config;
		raw.configFilePath = configFilePath;
	end
end

end