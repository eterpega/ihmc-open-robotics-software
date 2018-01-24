package us.ihmc.avatar.simulationStarter;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;

public interface HighLevelControllerConfiguration
{
   void configure(HighLevelHumanoidControllerFactory factory);
}
