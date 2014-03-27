function make(modelDimension)
if nargin < 1
    modelDimension = 2;
elseif modelDimension ~= 2 && modelDimension ~= 3
    error(['Model dimension must be either 2 or 3. No argument results ' ...
           'in dimension 2']);
end


builddir = fullfile('.', sprintf(...
    'build_old-model-%dd-point-feet-amber-suite', modelDimension));
%    'build_old-model-%dd-feet-amber-suite', modelDimension));

files = dir(fullfile(builddir, '*.cc'));

thisinc = fullfile(pwd, 'include');
eigeninc = getenv('EIGEN3_INC_DIR');
mathematicainc = getenv('MATHEMATICA_INC_DIR');

if isempty(eigeninc)
    eigeninc = '/share/include/eigen3/';
end

if isempty(mathematicainc)
    mathematicainc = ['/usr/local/Wolfram/Mathematica/8.0/' ...
                      'SystemFiles/IncludeFiles/C'];
end

for i = 1:length(files);
    fname = fullfile(builddir, files(i).name);

    mexCommand = sprintf(['mex -I%s ...\n      -I%s ...\n      -I%s ' ...
                        '...\n      -outdir %s ...\n      %s'], ...
                         thisinc, eigeninc, mathematicainc, ...
                         builddir, fname);

    fprintf(['[%d/%d] Compiling and linking MEX object from ''%s'' ' ...
             'with the following command:\n  %s\n'], ...
            i, length(files), fname, mexCommand)

    eval(mexCommand)
end
