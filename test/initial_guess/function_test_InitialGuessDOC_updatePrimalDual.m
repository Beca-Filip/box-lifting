function r = function_test_InitialGuessDOC_updatePrimalDual(igdoc)
%FUNCTION_TEST_INITIALGUESSDOC_UPDATEPRIMALDUAL calls the method
%updatePrimalDual on the InitialGuessDOC object that is passed as an 
%argument.

igdoc.updatePrimalDual(randi([1, igdoc.nTrials], [1, 1]), rand(igdoc.nPrimal, 1), rand(igdoc.nDual, 1));

r = rand(1, 1);

end

