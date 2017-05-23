package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.AvatarWalkThroughHatchBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasWalkThroughHatchBehaviorTest extends AvatarWalkThroughHatchBehaviorTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false); //ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 63.6, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test
   public void testWalkThroughHatch() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testWalkThroughHatch();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 1.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   //@Test
   public void testInitializeRobotCollisionModel() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testInitializeRobotCollisionModel();
   }
}
