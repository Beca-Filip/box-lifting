classdef BoxLiftingLocalIOC
    %LOCALIOC performs a local IOC search powered by fmincon.
    
    properties
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
            % Solution obj
            obj.solution = struct();
        end
        
        % Outer loop cost function
        J = costFun(obj, w);
        J = costFun_noInit(obj, w);
        
        % Runs the IOC
        obj = runIOC(obj, varargin);
        % Runs genetic algorithms on IOC
        obj = runGlobalIOC(obj, varargin);
        % Runs genetic algorithms on IOC
        obj = runGridIOC(obj, varargin);
        
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

