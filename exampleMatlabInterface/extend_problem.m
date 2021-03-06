classdef extend_problem < handle
    
    properties (Access = private)
        id_ % ID of the session
    end
    
    methods
        function this = extend_problem(Cptr)
            %extendProblem Create a new
            disp(Cptr)
            assert(isinteger(Cptr));
            this.id_ = extendProblem('new',Cptr);
        end
        
        function delete(this)
            %delete Destructor
            extendProblem('delete', this.id_);
        end
        function addAccelerationGoal(this,weight,coordNames, displacementDiv)
            assert(isscalar(weight));
            extendProblem('addAccelerationGoal', this.id_,weight, coordNames, displacementDiv)
        end
        function addMarkerGoal(this,weight,markerName,displacementDiv)
            assert(isscalar(weight));
            extendProblem('addMarkerGoal', this.id_,weight,markerName,displacementDiv)
        end

    end
end