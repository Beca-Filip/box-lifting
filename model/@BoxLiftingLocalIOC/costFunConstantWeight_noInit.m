function J = costFunConstantWeight_noInit(obj, w)
%COSTFUNCONSTANTWEIGHT_NOINIT implements the outer loop cost function for the weights w.

% Initial cost value
J = 0;

% Set the parameters of the DOC
obj.doc = obj.doc.setCostFunctionParameters(w);

% Do the DOC for every trial
for TrialIndex = 1 : length(obj.Trials)
    
    % Instantiate parameters of doc with trial
    if length(obj.Trials) ~= 1
        obj.doc = obj.doc.instantiateParameters(obj.Trials(TrialIndex).humanModel, obj.Trials(TrialIndex).liftingEnvironment, obj.Trials(TrialIndex).splineTrajectory);
    end
    
    % Set the initial guess for primal variables
    obj.doc.opti.set_initial(obj.doc.opti.x, obj.initGuessDOC.primal(:, TrialIndex));
    % Set the initial guess for dual variables
    obj.doc.opti.set_initial(obj.doc.opti.lam_g, obj.initGuessDOC.dual(:, TrialIndex));
    
    % Solve the DOC
    sol_doc = obj.doc.opti.solve();
    
    % Extract trajectory
    q_opt = sol_doc.value(obj.doc.q);
    % Extract human trajectory
    q_hum = obj.Trials(TrialIndex).splineTrajectory.currentEvaluatedValuesAndDerivatives{1};
    
    % Calculate the Cost
    J = J + mean(row_rmse(q_hum, q_opt));
end

end