boxliftingdoc = BoxLiftingDOC();

boxliftingdoc = boxliftingdoc.addJointLimitConstraints();
boxliftingdoc = boxliftingdoc.addInitialJointConstraints();
boxliftingdoc = boxliftingdoc.addFinalCartesianConstraints();