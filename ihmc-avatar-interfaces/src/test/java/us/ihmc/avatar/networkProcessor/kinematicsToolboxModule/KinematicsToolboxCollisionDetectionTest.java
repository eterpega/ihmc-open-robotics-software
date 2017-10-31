package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import static org.junit.Assert.assertTrue;
import static us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import gnu.trove.map.hash.THashMap;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.avatar.collisionAvoidance.RobotCollisionMeshProvider;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class KinematicsToolboxCollisionDetectionTest
{
   private static final boolean VERBOSE = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.YELLOW, 0.75);
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private KinematicsToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;
   private YoDouble finalSolutionQuality;

   private SimulationConstructionSet scs;
   private BlockingSimulationRunner blockingSimulationRunner;

   private Robot robot;
   private Robot ghost;
   private RobotController toolboxUpdater;

   private RobotDescription robotDescription;
   private JointNameMap robotJointMap;
   private FullRobotModel controllerFullRobotModel;
   private FrameConvexPolytopeVisualizer visualizer;

   @Before
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(true);
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      finalSolutionQuality = new YoDouble("finalSolutionQuality", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      robotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      robotJointMap = new KinematicsToolboxControllerTestRobots.SevenDoFArmJointMap();

      controllerFullRobotModel = new FullRobotModelFromDescription(robotDescription, robotJointMap, null);
      randomizeJointPositions(new Random(), controllerFullRobotModel, 1.0);
      commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(controllerFullRobotModel.getElevator()));

      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());

      visualizer = new FrameConvexPolytopeVisualizer(50, mainRegistry, yoGraphicsListRegistry);
      toolboxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, null, controllerFullRobotModel.getOneDoFJoints(),
                                                          yoGraphicsListRegistry, mainRegistry, visualizer);

      THashMap<RigidBody, FrameConvexPolytope> collisionMeshes = (new RobotCollisionMeshProvider(8)).createCollisionMeshesFromRobotDescription(controllerFullRobotModel,
                                                                                                                                               robotDescription);
      toolboxController.setCollisionMeshes(collisionMeshes);
      robot = new RobotFromDescription(robotDescription);
      new JointAnglesWriter(robot, controllerFullRobotModel.getRootJoint(), controllerFullRobotModel.getControllableOneDoFJoints());
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      RobotDescription ghostRobotDescription = new KinematicsToolboxControllerTestRobots.SevenDoFArm();
      ghostRobotDescription.setName("Ghost");
      recursivelyModyfyGraphics(ghostRobotDescription.getChildrenJoints().get(0));
      ghost = new RobotFromDescription(ghostRobotDescription);
      ghost.setDynamic(false);
      ghost.setGravity(0);

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot[] {robot, ghost}, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
      }
   }

   private void snapGhostToFullRobotModel(FullRobotModel fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel.getRootJoint(),
                            fullHumanoidRobotModel.getOneDoFJoints()).updateRobotConfigurationBasedOnFullRobotModel();
   }

   @After
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (mainRegistry != null)
      {
         mainRegistry.closeAndDispose();
         mainRegistry = null;
      }

      initializationSucceeded = null;

      yoGraphicsListRegistry = null;

      commandInputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;
      blockingSimulationRunner = null;

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   public void testHoldBodyPose() throws Exception
   {
      FullRobotModel initialFullRobotModel = new FullRobotModelFromDescription(robotDescription, robotJointMap, null);
      snapGhostToFullRobotModel(initialFullRobotModel);

      RigidBody hand = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(initialFullRobotModel.getRootJoint()), "handLink")[0];
      commandInputManager.submitMessage(holdRigidBodyCurrentPose(hand));

      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      int numberOfIterations = 2;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      assertTrue("Poor solution quality: " + toolboxController.getSolution().getSolutionQuality(),
                 toolboxController.getSolution().getSolutionQuality() < 1.0e-4);
   }

   @Test(timeout = 10)
   public void testHandCollsion() throws Exception
   {
      if (VERBOSE)
         PrintTools.info(this, "Entering: testRandomHandPositions");
      FrameConvexPolytope obstacle = ConvexPolytopeConstructor.getFrameCuboidCollisionMesh(ReferenceFrame.getWorldFrame(), new Point3D(0.15, 0.0, 0.65), 0.15,
                                                                                           0.15, 0.15);
      toolboxController.submitObstacleCollisionMesh(obstacle);
      Random random = new Random(2135);
      FullRobotModel initialFullRobotModel = new FullRobotModelFromDescription(robotDescription, robotJointMap, null);
      RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);

      FullRobotModel desiredFullRobotModel = createFullRobotModelAtInitialConfiguration();
      OneDoFJoint[] inputAngles = desiredFullRobotModel.getOneDoFJoints();
      inputAngles[0].setQ(Math.toRadians(0));
      inputAngles[1].setQ(Math.toRadians(0));
      inputAngles[2].setQ(Math.toRadians(90));
      inputAngles[3].setQ(Math.toRadians(-90));
      inputAngles[4].setQ(Math.toRadians(45));
      inputAngles[5].setQ(Math.toRadians(45));
      inputAngles[6].setQ(Math.toRadians(45));

      toolboxController.enableCollisionAvoidance(true);
      RigidBody hand = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(desiredFullRobotModel.getOneDoFJoints()), "handLink")[0];
      FramePoint3D desiredPosition = new FramePoint3D(hand.getBodyFixedFrame());
      desiredPosition.changeFrame(worldFrame);
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(hand, desiredPosition);
      message.setWeight(20.0);
      commandInputManager.submitMessage(message);

      snapGhostToFullRobotModel(desiredFullRobotModel);
      toolboxController.updateRobotConfigurationData(robotConfigurationData);

      int numberOfIterations = 200;

      runKinematicsToolboxController(numberOfIterations);

      assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
      double solutionQuality = toolboxController.getSolution().getSolutionQuality();
      if (VERBOSE)
         PrintTools.info(this, "Solution quality: " + solutionQuality);
      assertTrue("Poor solution quality: " + solutionQuality, solutionQuality < 1.0e-4);
   }

   @Test
   public void testRandomHandPosition() throws SimulationExceededMaximumTimeException
   {
      FramePoint3D origin = new FramePoint3D(worldFrame);
      for (int i = 0; i < 5; i++)
      {
         FullRobotModel initialFullRobotModel = new FullRobotModelFromDescription(robotDescription, robotJointMap, null);

         FullRobotModel desiredFullRobotModel = createFullRobotModelAtInitialConfiguration();
         randomizeJointPositions(new Random(), desiredFullRobotModel, 0.6);

         toolboxController.enableCollisionAvoidance(false);
         RigidBody desiredHand = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(desiredFullRobotModel.getOneDoFJoints()),
                                                                     "handLink")[0];
         FramePoint3D desiredPosition = new FramePoint3D(desiredHand.getBodyFixedFrame());
         desiredPosition.changeFrame(worldFrame);
         KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(desiredHand, desiredPosition);
         message.setWeight(20.0);
         commandInputManager.submitMessage(message);

         snapGhostToFullRobotModel(desiredFullRobotModel);
         randomizeJointPositions(new Random(), initialFullRobotModel, 0.6);
         RobotConfigurationData robotConfigurationData = extractRobotConfigurationData(initialFullRobotModel);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);

         int numberOfIterations = 200;
         runKinematicsToolboxController(numberOfIterations);

         RigidBody lowerArm = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(controllerFullRobotModel.getOneDoFJoints()),
                                                                  "lowerArmLink")[0];
         RigidBody hand = ScrewTools.findRigidBodiesWithNames(ScrewTools.computeRigidBodiesAfterThisJoint(controllerFullRobotModel.getOneDoFJoints()),
                                                              "handLink")[0];
         FramePoint3D pointForObstacle = new FramePoint3D(lowerArm.getBodyFixedFrame());
         //pointForObstacle.changeFrame(hand.getBodyFixedFrame());
         //pointForObstacle.scale(0.5);
         pointForObstacle.changeFrame(worldFrame);
         FrameConvexPolytope obstacleMesh = ConvexPolytopeConstructor.getFrameSphericalCollisionMeshByProjectingCube(pointForObstacle, 0.075, 4);
         toolboxController.clearObstacleMeshes();
         toolboxController.submitObstacleCollisionMesh(obstacleMesh);

         toolboxController.enableCollisionAvoidance(true);
         toolboxController.updateRobotConfigurationData(robotConfigurationData);
         commandInputManager.submitMessage(message);
         runKinematicsToolboxController(numberOfIterations * 2);

         System.gc();
         assertTrue(KinematicsToolboxController.class.getSimpleName() + " did not manage to initialize.", initializationSucceeded.getBooleanValue());
         double solutionQuality = toolboxController.getSolution().getSolutionQuality();
         if (VERBOSE)
            PrintTools.info(this, "Solution quality: " + solutionQuality);
         assertTrue("Poor solution quality: " + solutionQuality, solutionQuality < 1.0e-4);
      }
   }

   private void randomizeJointPositions(Random random, FullRobotModel randomizedFullRobotModel, double percentOfMotionRangeAllowed)
   {
      OneDoFJoint[] joints = randomizedFullRobotModel.getControllableOneDoFJoints();
      for (OneDoFJoint joint : joints)
      {
         double jointLimitLower = joint.getJointLimitLower();
         if (Double.isInfinite(jointLimitLower))
            jointLimitLower = -Math.PI;
         double jointLimitUpper = joint.getJointLimitUpper();
         if (Double.isInfinite(jointLimitUpper))
            jointLimitUpper = -Math.PI;
         double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
         jointLimitLower += 0.5 * rangeReduction;
         jointLimitUpper -= 0.5 * rangeReduction;
         joint.setQ(RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper));
      }
   }

   private FullRobotModel createFullRobotModelAtInitialConfiguration()
   {
      FullRobotModel robotModel = new FullRobotModelFromDescription(robotDescription, robotJointMap, null);
      for (OneDoFJoint joint : robotModel.getControllableOneDoFJoints())
      {
         double lowerLimit = joint.getJointLimitLower();
         double upperLimit = joint.getJointLimitUpper();
         joint.setQ((upperLimit + lowerLimit) / 2.0);
      }
      return robotModel;
   }

   private void runKinematicsToolboxController(int numberOfIterations) throws SimulationExceededMaximumTimeException
   {
      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         blockingSimulationRunner.simulateNTicksAndBlockAndCatchExceptions(numberOfIterations);
      }
      else
      {
         for (int i = 0; i < numberOfIterations; i++)
            toolboxUpdater.doControl();
      }

      finalSolutionQuality.set(toolboxController.getSolution().getSolutionQuality());
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, controllerFullRobotModel.getRootJoint(),
                                                                                   toolboxController.getDesiredOneDoFJoint());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               toolboxController.updateInternal();
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               numberOfIterations.increment();
               visualizer.update();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }

   private static void recursivelyModyfyGraphics(JointDescription joint)
   {
      if (joint == null)
         return;
      LinkDescription link = joint.getLink();
      if (link == null)
         return;
      LinkGraphicsDescription linkGraphics = link.getLinkGraphics();

      if (linkGraphics != null)
      {
         ArrayList<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

         if (graphics3dInstructions == null)
            return;

         for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
         {
            if (primitive instanceof Graphics3DInstruction)
            {
               Graphics3DInstruction modelInstruction = (Graphics3DInstruction) primitive;
               modelInstruction.setAppearance(ghostApperance);
            }
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModyfyGraphics(child);
      }
   }

   private RobotConfigurationData extractRobotConfigurationData(FullRobotModel initialFullRobotModel)
   {
      OneDoFJoint[] joints = initialFullRobotModel.getOneDoFJoints();
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData(joints, new ForceSensorDefinition[0], null, new IMUDefinition[0]);
      robotConfigurationData.setJointState(Arrays.stream(joints).collect(Collectors.toList()));

      FloatingInverseDynamicsJoint rootJoint = initialFullRobotModel.getRootJoint();
      if (rootJoint != null)
      {
         robotConfigurationData.setRootTranslation(rootJoint.getTranslationForReading());
         robotConfigurationData.setRootOrientation(rootJoint.getRotationForReading());
      }
      return robotConfigurationData;
   }
}
