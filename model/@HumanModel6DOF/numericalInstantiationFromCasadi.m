function numericHumanModel = numericalInstantiationFromCasadi(obj, solutionObj)
    
    
    % Initialize it
    numericHumanModel = HumanModel6DOF();
    %% Extract all parameters of a human model
    % Parameters 
    numericHumanModel.HEIGHT = solutionObj.value(obj.HEIGHT);   
    numericHumanModel.WEIGHT = solutionObj.value(obj.WEIGHT);

    numericHumanModel.R = solutionObj.value(obj.R);
    numericHumanModel.p = solutionObj.value(obj.p);

    numericHumanModel.HeelPosition = solutionObj.value(obj.HeelPosition);
    numericHumanModel.ToePosition = solutionObj.value(obj.ToePosition);

    numericHumanModel.L = solutionObj.value(obj.L);
    numericHumanModel.CoM = solutionObj.value(obj.CoM);
    numericHumanModel.M = solutionObj.value(obj.M);
    numericHumanModel.Izz = solutionObj.value(obj.Izz);

    numericHumanModel.LFOOT = solutionObj.value(obj.LFOOT);
    numericHumanModel.LHAND = solutionObj.value(obj.LHAND);
    numericHumanModel.LHEAD = solutionObj.value(obj.LHEAD);
    numericHumanModel.CoMFOOT = solutionObj.value(obj.CoMFOOT);
    numericHumanModel.CoMHAND = solutionObj.value(obj.CoMHAND);
    numericHumanModel.CoMHEAD = solutionObj.value(obj.CoMHEAD);
    numericHumanModel.MFOOT = solutionObj.value(obj.MFOOT);
    numericHumanModel.MHAND = solutionObj.value(obj.MHAND);
    numericHumanModel.MHEAD = solutionObj.value(obj.MHEAD);
    numericHumanModel.IzzFOOT = solutionObj.value(obj.IzzFOOT);
    numericHumanModel.IzzHAND = solutionObj.value(obj.IzzHAND);
    numericHumanModel.IzzHEAD = solutionObj.value(obj.IzzHEAD);

    numericHumanModel.LowerJointLimits = solutionObj.value(obj.LowerJointLimits);
    numericHumanModel.UpperJointLimits = solutionObj.value(obj.UpperJointLimits);
    numericHumanModel.LowerTorqueLimits = solutionObj.value(obj.LowerTorqueLimits);
    numericHumanModel.UpperTorqueLimits = solutionObj.value(obj.UpperTorqueLimits);

    numericHumanModel.LengthToRadiiFactor = solutionObj.value(obj.LengthToRadiiFactor);
    
    for numCs = 1 : length(obj.CS)
        % Extract collision sphere params
        Name = obj.CS(numCs).Name;
        RigidlyLinkedTo = obj.CS(numCs).RigidlyLinkedTo;
        PositionExpressedInFrame = obj.CS(numCs).PositionExpressedInFrame;
        p = solutionObj.value(obj.CS(numCs).p);
        radius = solutionObj.value(obj.CS(numCs).radius);
        % Create sphere
        collSphere = CollisionSphere6DOF(Name, RigidlyLinkedTo, PositionExpressedInFrame, p, radius);
        % Add sphere
        numericHumanModel = numericHumanModel.addCollisionSphere(collSphere);
    end
    
    for numKpoi = 1 : length(obj.KPOI)
        % Extract kinematic points of interest params
        Name = obj.KPOI(numKpoi).Name;
        RigidlyLinkedTo = obj.KPOI(numKpoi).RigidlyLinkedTo;
        PositionExpressedInFrame = obj.KPOI(numKpoi).PositionExpressedInFrame;
        p = solutionObj.value(obj.KPOI(numKpoi).p);
        % Create kinematic points of interest
        kinpoint = KinematicPointOfInterest6DOF(Name, RigidlyLinkedTo, PositionExpressedInFrame, p);
        % Add kinematic points of interest
        numericHumanModel = numericHumanModel.addKinematicPointOfIntrerest(kinpoint);
    end
         
end