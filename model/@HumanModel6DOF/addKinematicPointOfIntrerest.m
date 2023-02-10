function obj = addKinematicPointOfIntrerest(obj, kpoi)
%ADDKINEMATICPOINTOFINTREREST adds a particular point of interest to the model.
    obj.KPOI = [obj.KPOI, kpoi];
end