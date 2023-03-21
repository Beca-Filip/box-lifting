function printToFile(obj, filename)

% Open file
fileID = fopen(filename, 'wt');

% All properties of the obj
PropertyNames = properties(obj);
% Go through each property
for ii = 1 : length(PropertyNames)
    
    % Get the property name
    propertyname = PropertyNames{ii};
    
    % Get the property
    property = obj.(propertyname);
    
    % Get the length of the property
    propertylength = length(property);
    
    % Get string format for property
    if propertylength > 1
        stringformat = "%s: %f" + join(repmat(", %f", [1, propertylength-1]));
    else
        stringformat = "%s: %f";
    end
    
    % Get whole string for property
    stringproperty = sprintf(stringformat, propertyname, property);
    
    % Print to file
    fprintf(fileID, stringproperty + "\n");
end

% Close file
fclose(fileID);

end