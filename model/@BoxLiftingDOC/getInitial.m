function val = getInitial(obj, cas)
%GETINITIAL gets the inital value fo some quantity of the DOC. it is the
%wrapper of the following expression:
%   obj.opti.debug.value(cas, obj.opti.initial())
val = obj.opti.debug.value(cas, obj.opti.initial());
end