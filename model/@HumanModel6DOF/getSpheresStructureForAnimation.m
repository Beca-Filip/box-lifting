function spheres = getSpheresStructureForAnimation(obj)

spheres.centers = [obj.CS.p];
spheres.radii = [obj.CS.radius];
spheres.parent_segment = [obj.CS.PositionExpressedInFrame];
end