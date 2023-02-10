function [WristToBoxGripPointVector] = defaultGripPointParameters(obj)
%DEFAULTGRIPPOINTPARAMETERS returns the default wrist to grip position 
%parameters for the lifting task.
%
%The numerical values are consistent with the experimental results.
%
%   [WristToBoxGripPointVector] = DEFAULTGRIPPOINTPARAMETERS(obj)
%   

% Default values
WristToBoxGripPointVector = [0.1044; -0.0357];
end

