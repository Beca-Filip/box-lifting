function printToFile(obj, filename)

% Lengths and BSIPs
var1 = [obj.LFOOT; reshape(obj.L, [], 1); obj.LHAND; obj.LHEAD];
var2 = [obj.MFOOT; reshape(obj.M, [], 1); obj.MHAND; obj.MHEAD];
var3 = [obj.CoMFOOT(1); reshape(obj.CoM(1, :), [], 1); obj.CoMHAND(1); obj.CoMHEAD(1)];
var4 = [obj.CoMFOOT(2); reshape(obj.CoM(2, :), [], 1); obj.CoMHAND(2); obj.CoMHEAD(2)];
var5 = [obj.IzzFOOT; reshape(obj.Izz, [], 1); obj.IzzHAND; obj.IzzHEAD];
PropertyNames = {'Length [m]', 'Mass [kg]', 'Center of Mass X-Position [m]', 'Center of Mass Y-Position [m]', 'Inertia ZZ [kg.m^2]'};
RowNames = obj.TreeLinkNames;
% Create table
t = table(var1, var2, var3, var4, var5, 'VariableNames', PropertyNames, 'RowNames', RowNames);
% Write table to file
writetable(t, filename, 'WriteRowNames', true);

% Heel and Toe positions
var1 = vertcat(obj.HeelPosition(1), obj.ToePosition(1));
var2 = vertcat(obj.HeelPosition(2), obj.ToePosition(2));
PropertyNames = {'X-Position [m]', 'Y-Position [m]'};
RowNames = {'Heel', 'Toe'};
% Create table
t = table(var1, var2, 'VariableNames', PropertyNames, 'RowNames', RowNames);
% Write table to file
writetable(t, filename, 'WriteRowNames', true, 'WriteMode', 'append', 'WriteVariableNames', true);

end