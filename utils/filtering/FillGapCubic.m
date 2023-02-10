function signal=FillGapCubic(signal)

%Bonnet July 2011

signal(isnan(signal))=interp1(find(~isnan(signal)),...
   signal(~isnan(signal)), find(isnan(signal)), 'PCHIP');

return