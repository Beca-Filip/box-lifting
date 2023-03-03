classdef InitialGuessDOC < handle
    %INITIALGUESSDOC handle class for DOC initial guesses
    
    properties
        % Number of trials
        nTrials(1, 1)
        % Size of primal solution
        nPrimal(1, 1)
        % Size of dual solution
        nDual(1, 1)
        % Guess for primal solution
        primal
        % Guess for dual solution
        dual
    end
    
    methods
        function obj = InitialGuessDOC(nTrials, nPrimal, nDual)
            % Get the numer of trials
            obj.nTrials = nTrials;
            % Initialize the size of primal solution
            obj.nPrimal = nPrimal;
            % Initialize the size of dual solution
            obj.nDual = nDual;
            
            % Initialize the primal solution
            obj.primal = zeros(obj.nPrimal, obj.nTrials);
            % Initialize the dual solution
            obj.dual = zeros(obj.nDual, obj.nTrials);
        end
        
        function updatePrimalDual(obj, indTrial, currPrimal, currDual)
            
            % Update the current primal and current dual given by the index
            % of the Trial
            obj.primal(:, indTrial) = currPrimal;
            obj.dual(:, indTrial) = currDual;
        end
    end
end

