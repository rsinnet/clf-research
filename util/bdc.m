%%%BDC Converts a binary vector to an integer (decimal).
% This function replaces the functionality of the bi2de function
% from the communications toolbox.
function dval = bdc(bvec)

dval = bin2dec(arrayfun(@(bit) int2str(bit), bvec));
