classdef BoxLiftingLocalIOC
    %LOCALIOC performs a local IOC search powered by fmincon.
    
    properties
        % Some fixed weights the IOC won't search for
        fixedWeights(:, 1)
        % Experiment trials structure
        Trials(:, 1)        struct
        % Handle object to store current guess for primal and dual doc solution
        initGuessDOC(1, 1)  InitialGuessDOC
        % DOC object
        doc(1, 1)           BoxLiftingDOC
        % Solution object
        solution(1, 1)      struct
    end
    
    methods
        function obj = BoxLiftingLocalIOC(Trials, doc)
            % Copy trials
            obj.Trials = Trials;
            % Copy DOC
            obj.doc = doc;
            % Extract parameters for constructor of initial guess
            nTrials = length(obj.Trials);
            nPrimal = length(obj.doc.opti.x);
            nDual = length(obj.doc.opti.lam_g);
            % Create an initial guess            
            obj.initGuessDOC = InitialGuessDOC(nTrials, nPrimal, nDual);
            % Initialize primal guesses using Trials and DOC
            for TrialIndex = 1 : length(obj.Trials)
                % Instantiate DOC parameters using Trials
                obj.doc = obj.doc.instantiateParameters(obj.Trials(TrialIndex).humanModel, obj.Trials(TrialIndex).liftingEnvironment, obj.Trials(TrialIndex).splineTrajectory);
                % Initialize primal solution
                obj.initGuessDOC.primal(:, TrialIndex) = obj.doc.opti.debug.value(obj.doc.opti.x, obj.doc.opti.initial());
            end
            % Initialize fixed weights to empty
            obj.fixedWeights = [];
            % Solution obj
            obj.solution = struct();
        end
        
        % Outer loop cost function
        J = costFun(obj, w);
        J = costFun_noInit(obj, w);
        J = costFunSomeFixedWeights(obj, w);
        J = costFunConstantWeight(obj, w);
        J = costFunConstantWeight_noInit(obj, w);
        
        % Runs constant weight IOC
        obj = runConstantWeightIOC(obj, varargin);
        % Runs genetic algorithms on constant weight IOC
        obj = runConstantWeightGlobalIOC(obj, varargin);
        % Runs constant weight grid IOC
        obj = runConstantWeightGridIOC(obj, varargin);
        % Runs the IOC
        obj = runIOC(obj, varargin);
        % Runs genetic algorithms on IOC
        obj = runGlobalIOC(obj, varargin);
        % Runs genetic algorithms on IOC
        obj = runGridIOC(obj, varargin);
        % Runs the IOC with some fixed weights
        obj = runIOCsomeFixedWeights(obj, varargin);
        
        % Add the fixed weights to currently considered vector
        w = addFixedWeights(obj, w);
        % Vectorize the weights
        w = vectorizeWeightsForIOC(obj, w);
        % Matricize the weights for IOC
        w = matricizeWeightsForIOC(obj, w);
        % Expand the weights
        w = expandWeightsForDOC(obj, w);
    end
    
    methods (Static)
        % Initialization of weights
        w0 = getInitialWeightsIOC(nCF, nT);
    end
end

