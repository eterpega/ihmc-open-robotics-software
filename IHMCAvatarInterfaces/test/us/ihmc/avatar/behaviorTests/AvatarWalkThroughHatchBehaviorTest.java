package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.FiducialDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughHatchBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HatchLocationPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.Hatch;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarWalkThroughHatchBehaviorTest implements MultiRobotTestInterface
{
   private static final HatchEnvironment ENVIRONMENT = new HatchEnvironment();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setKeepSCSUp(true);
   }
   
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
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      drcRobotModel = getRobotModel();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(ENVIRONMENT, getSimpleRobotName(), DRCObstacleCourseStartingLocation.DEFAULT,
                                                        simulationTestingParameters, drcRobotModel);
   }

   public void testWalkThroughHatch() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      PrintTools.debug("Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      drcBehaviorTestHelper.updateRobotModel();
      
      PrintTools.debug("Initializing Behavior");

      CommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      HumanoidReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      YoDouble yoTime = drcBehaviorTestHelper.getYoTime();
      YoVariableRegistry scsRootRegistry = drcBehaviorTestHelper.getSimulationConstructionSet().getRootRegistry();
      
      CapturePointUpdatable capturePointUpdatable = drcBehaviorTestHelper.getCapturePointUpdatable();
      YoBoolean yoDoubleSupport = capturePointUpdatable.getYoDoubleSupport();
      
      FiducialDetectorBehaviorService fiducialDetectorBehaviorService = new FiducialDetectorBehaviorService(communicationBridge, yoGraphicsListRegistry);
      fiducialDetectorBehaviorService.setTargetIDToLocate(50);
      
      AtlasPrimitiveActions primitiveActions = new AtlasPrimitiveActions(communicationBridge, fullRobotModel, referenceFrames, drcRobotModel.getWalkingControllerParameters(),
                                                                         yoTime, drcRobotModel, scsRootRegistry);      
      
      WalkThroughHatchBehavior walkThroughHatchBehavior = new WalkThroughHatchBehavior(communicationBridge, yoTime, yoDoubleSupport, fullRobotModel,
                                                                                       referenceFrames, drcRobotModel, primitiveActions, yoGraphicsListRegistry);      
//      walkThroughHatchBehavior.initialize();
      drcBehaviorTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);

      drcBehaviorTestHelper.addChildRegistry(fiducialDetectorBehaviorService.getYoVariableRegistry());
      drcBehaviorTestHelper.addChildRegistry(walkThroughHatchBehavior.getYoVariableRegistry());

      PrintTools.info(this, "Starting to Execute Behavior");
      
      for (int i = 0; i < HatchEnvironment.getNumberOfHatches(); i++)
      {
         walkThroughHatchBehavior.initialize();
         
         Hatch hatch = HatchEnvironment.getHatch(i);
         RigidBodyTransform hatchToWorldTransform = hatch.getHatchToWorldTransform();
         communicationBridge.sendPacketToBehavior(new HatchLocationPacket(hatchToWorldTransform, hatch.getStepHeight(), hatch.getOpeningHeight(), hatch.getWidth(), hatch.getThickness()));
         communicationBridge.sendPacketToBehavior(new HatchLocationPacket(hatchToWorldTransform, hatch.getStepHeight(), hatch.getOpeningHeight(), hatch.getWidth(), hatch.getThickness()));
         communicationBridge.sendPacketToBehavior(new HatchLocationPacket(hatchToWorldTransform, hatch.getStepHeight(), hatch.getOpeningHeight(), hatch.getWidth(), hatch.getThickness()));
         
         success = drcBehaviorTestHelper.executeBehaviorUntilDone(walkThroughHatchBehavior);
         assertTrue(success);
         PrintTools.info("Behavior is Done");
                  
         assertTrue(walkThroughHatchBehavior.isDone());
      }
      
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
   
   public void testInitializeRobotCollisionModel() throws SimulationExceededMaximumTimeException
   {  
      PrintTools.debug("Initializing Simulation");
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      drcBehaviorTestHelper.updateRobotModel();
      
      PrintTools.debug("Initializing Robot Collision Model");
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();      
      RobotCollisionModel robotCollisionModel = new RobotCollisionModel(sdfFullRobotModel);
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      scs.addStaticLinkGraphics(robotCollisionModel.getCollisionGraphics());
      
      PrintTools.info("Initializing Robot Collision Model is Done" );      
   }
}
