function printToFile(obj, filename)

% All properties of the obj
PropertyNames = properties(obj);
% All vars
var = cell(length(PropertyNames), 1);
% Go through each property
for ii = 1 : length(PropertyNames)
    
    % Get the property name
    propertyname = PropertyNames{ii};
    
    % Get the property
    property = obj.(propertyname);
    
    % Set var
    var{ii} = reshape(property, 1, []);    
end

% Create table
t = table(var{:}, 'VariableNames', PropertyNames);
% Write table to file
writetable(t, filename, 'WriteRowNames', false);

end