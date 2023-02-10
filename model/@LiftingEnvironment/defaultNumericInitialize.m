function obj = defaultNumericInitialize(obj)
%DEFAULTNUMERICINITIALIZE returns a LiftingExperiment object with default
%numeric initializers.
%
%The numerical values are consistent with the experiment design and
%experimental results.
%
%   obj = DEFAULTNUMERICINITIALIZE(obj)
%   

% Get all default values
[BoxWidth, BoxHeight, BoxMass] = defaultBoxParameters(obj);
[TableWidth, TableHeight, TableCenterCoordinates] = defaultTableParameters(obj);
[WristInitialPosition, WristFinalPosition] = defaultWristPositionParameters(obj);
[WristToBoxGripPointVector] = defaultGripPointParameters(obj);

% Requires model knowledge so initialize to zeros
JointAnglesInitial = zeros(6, 1);
JointAnglesFinal = zeros(6, 1);

% Just store them in the object
obj.BoxWidth = BoxWidth;
obj.BoxHeight = BoxHeight;
obj.BoxMass = BoxMass;
obj.TableWidth = TableWidth;
obj.TableHeight = TableHeight;
obj.TableCenterCoordinates = TableCenterCoordinates;
obj.WristInitialPosition = WristInitialPosition;
obj.WristFinalPosition = WristFinalPosition;
obj.WristToBoxGripPointVector = WristToBoxGripPointVector;
obj.JointAnglesInitial = JointAnglesInitial;
obj.JointAnglesFinal = JointAnglesFinal;
end

