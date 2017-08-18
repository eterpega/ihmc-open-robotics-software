package us.ihmc.avatar.controllerAPI;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public abstract class WalkingStatusMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCNetworkModuleParameters networkModuleParameters;

   private AtomicInteger startedCount = new AtomicInteger(0);
   private AtomicInteger completedCount = new AtomicInteger(0);
   private AtomicInteger abortCount = new AtomicInteger(0);

   private AtomicReference<WalkingStatusMessage> lastStatusReceived = new AtomicReference<>(null);

   @Before
   public void setup()
   {
      networkModuleParameters = new DRCNetworkModuleParameters();

      networkModuleParameters.enableLocalControllerCommunicator(true);

      lastStatusReceived.set(null);
   }

   @After
   public void tearDown()
   {
      networkModuleParameters = null;
      drcSimulationTestHelper.destroySimulation();
      drcSimulationTestHelper = null;

      lastStatusReceived.set(null);
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test
   public void testOnlyReceiveStatusMessagesOnce() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(new FlatGroundEnvironment(), "steppingStonesTestHelper", OffsetAndYawRobotInitialSetup::new,
                                                            simulationTestingParameters, robotModel, networkModuleParameters);

      drcSimulationTestHelper.getControllerCommunicator().connect();

      drcSimulationTestHelper.getControllerCommunicator().attachListener(WalkingStatusMessage.class, this::listenForWalkingStatusMessages);

      BlockingSimulationRunner blockingSimulationRunner = drcSimulationTestHelper.getBlockingSimulationRunner();

      // let everything get setup and swallow the first COMPLETED status
      try
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
      }
      catch (BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         e.printStackTrace();
         fail(e.getMessage());
      }

      startedCount.set(0);
      completedCount.set(0);
      abortCount.set(0);

      FootstepDataListMessage message = new FootstepDataListMessage();
      int numberOfSteps = 4;
      double stepLength = 0.4;
      double stepWidth = 0.4;

      RobotSide side = RobotSide.LEFT;
      for (int i = 0; i < numberOfSteps; i++)
      {
         Point3D footLocation = new Point3D(stepLength * i, side.negateIfRightSide(stepWidth / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);

         FootstepDataMessage footstepDataMessage = new FootstepDataMessage();
         footstepDataMessage.setLocation(footLocation);
         footstepDataMessage.setOrientation(footOrientation);
         footstepDataMessage.setRobotSide(side);
         message.add(footstepDataMessage);

         side = side.getOppositeSide();
      }

      drcSimulationTestHelper.send(message);

      try
      {
         blockingSimulationRunner.simulateAndBlock(1.0);
      }
      catch (BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         e.printStackTrace();
         fail(e.getMessage());
      }

      assertEquals("Received more than one STARTED walking status message!", 1, startedCount.get());
      assertTrue("Started plan but last status received is not STARTED!", lastStatusReceived.get().status == WalkingStatusMessage.Status.STARTED);

      while(lastStatusReceived.get().status != WalkingStatusMessage.Status.COMPLETED)
      {
         try
         {
            blockingSimulationRunner.simulateAndBlock(1.0);
         }
         catch (BlockingSimulationRunner.SimulationExceededMaximumTimeException | ControllerFailureException e)
         {
            e.printStackTrace();
            fail(e.getMessage());
         }
      }

      assertEquals("Received more than one COMPLETED walking status message!", 1, completedCount.get());
      assertTrue("Plan completed but last status received no longer COMPLETED!", lastStatusReceived.get().status == WalkingStatusMessage.Status.COMPLETED);
   }

   private void listenForWalkingStatusMessages(WalkingStatusMessage statusMessage)
   {
      switch (statusMessage.status)
      {

      case STARTED:
         startedCount.incrementAndGet();
         break;
      case COMPLETED:
         completedCount.incrementAndGet();
         break;
      case ABORT_REQUESTED:
         abortCount.incrementAndGet();
         break;
      }

      lastStatusReceived.set(statusMessage);
   }
}
