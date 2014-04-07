classdef handle < hsys.controller.controller
%%%CONTROLLER Default controller template.
    properties (Abstract)
        Name;
        gains;
    end

    methods (abstract)
        function this =  controller();
    end
end
