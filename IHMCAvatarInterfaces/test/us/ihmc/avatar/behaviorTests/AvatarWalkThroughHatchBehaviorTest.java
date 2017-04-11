package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughHatchBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public abstract class AvatarWalkThroughHatchBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private DRCRobotModel drcRobotModel;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCWalkToLocationBehaviorTest.class + " after class.");
   }

   @Before
   public void setUp()
   {
      drcRobotModel = getRobotModel();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(new HatchEnvironment(), getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, drcRobotModel);
   }

   public void testWalkThroughHatch() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         simulationTestingParameters.setKeepSCSUp(true);
      }

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      PrintTools.debug(this, "Initializing Sim");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      drcBehaviorTestHelper.updateRobotModel();
      PrintTools.debug(this, "Initializing Behavior");

      CommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();
      YoVariableRegistry scsRootRegistry = drcBehaviorTestHelper.getSimulationConstructionSet().getRootRegistry();
      
      CapturePointUpdatable capturePointUpdatable = drcBehaviorTestHelper.getCapturePointUpdatable();
      BooleanYoVariable yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      
      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(communicationBridge, yoGraphicsListRegistry);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      
      AtlasPrimitiveActions primitiveActions = new AtlasPrimitiveActions(communicationBridge, fullRobotModel, referenceFrames, drcRobotModel.getWalkingControllerParameters(),
                                                                         yoTime, drcRobotModel, scsRootRegistry);      
      
      WalkThroughHatchBehavior walkThroughHatchBehavior = new WalkThroughHatchBehavior(communicationBridge, yoTime, yoDoubleSupport, fullRobotModel,
                                                                                       referenceFrames, drcRobotModel, primitiveActions);      
      walkThroughHatchBehavior.initialize();
      drcBehaviorTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      drcBehaviorTestHelper.addChildRegistry(fiducialDetectorBehaviorService.getYoVariableRegistry());
      drcBehaviorTestHelper.addChildRegistry(walkThroughHatchBehavior.getYoVariableRegistry());

      PrintTools.debug(this, "Starting to Execute Behavior");
      
      
//      drcBehaviorTestHelper.dispatchBehavior(walkThroughHatchBehavior);
      
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkThroughHatchBehavior);
      assertTrue(success);
      PrintTools.debug(this, "Behavior Should be done");

      assertTrue(walkThroughHatchBehavior.isDone());

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
