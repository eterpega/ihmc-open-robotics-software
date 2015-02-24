package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifier;
import us.ihmc.simulationconstructionset.util.simulationRunner.SimulationRewindabilityVerifierWithStackTracing;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.BoundingBox3d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.trajectories.TrajectoryType;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCObstacleCoursePlatformTest implements MultiRobotTestInterface
{   
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Ignore
   @QuarantinedTest("This test is flaky. Sometimes it works, sometimes it doesn't due to threading of the various globalDataProducer and communicators. We need to be able to shut those off or make them not screw up the robot run.")
   @AverageDuration
   @Test(timeout=300000)
   public void testRunsTheSameWayTwiceJustStanding() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      simulationTestingParameters.setRunMultiThreaded(false);
      
      DRCSimulationTestHelper drcSimulationTestHelper1 = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());
      DRCSimulationTestHelper drcSimulationTestHelper2 = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());
      
      ArrayList<String> exceptions = DRCSimulationTestHelper.createVariableNamesStringsToIgnore();
      
      SimulationConstructionSet scs1 = drcSimulationTestHelper1.getSimulationConstructionSet();
      SimulationConstructionSet scs2 = drcSimulationTestHelper2.getSimulationConstructionSet();
      SimulationRewindabilityVerifier checker = new SimulationRewindabilityVerifier(scs1, scs2, exceptions);
      
//      setupCameraForWalkingOverSmallPlatform(scs1);
//      setupCameraForWalkingOverSmallPlatform(scs2);

      SimulationRewindabilityVerifierWithStackTracing helper = null;
      boolean useVariableListenerTestHelper = false; // Make this true if you want to record a history of variable changes and find the first instance where they are different. Don't check in as true though since takes lots of time.

      if (useVariableListenerTestHelper)
      {
         helper = new SimulationRewindabilityVerifierWithStackTracing(scs1, scs2, exceptions);
         helper.setRecordDifferencesForSimOne(true);
         helper.setRecordDifferencesForSimTwo(true);
         helper.clearChangesForSimulations();
      }
      
      if (useVariableListenerTestHelper)
      {
         int numberOfTicks = 10;
         for (int i=0; i<numberOfTicks ; i++)
         {
            System.out.println("Tick : " + i);
            scs1.simulateOneRecordStepNow();
            scs2.simulateOneRecordStepNow();

            boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
            if (areTheVariableChangesDifferent) helper.printOutStackTracesOfFirstChangedVariable();
         }
      
      }
      
      double runTime = 10.18;

      boolean success = drcSimulationTestHelper1.simulateAndBlockAndCatchExceptions(runTime);
      success = success && drcSimulationTestHelper2.simulateAndBlockAndCatchExceptions(runTime);

      if (useVariableListenerTestHelper)
      {
         System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityHelper");

         boolean areTheVariableChangesDifferent = helper.areTheVariableChangesDifferent();
         if (areTheVariableChangesDifferent) helper.printOutStackTracesOfFirstChangedVariable();
      }

      System.out.println("Checking for variable differences at the end of the run using SimulationRewindabilityVerifier");
      ArrayList<VariableDifference> variableDifferences = checker.verifySimulationsAreSameToStart();

      if (!variableDifferences.isEmpty())
      {
         System.err.println("variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
         if (simulationTestingParameters.getKeepSCSUp())
            ThreadTools.sleepForever();
         fail("Found Variable Differences!\n variableDifferences: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences));
      }


      drcSimulationTestHelper1.destroySimulation();
      drcSimulationTestHelper2.destroySimulation();

      BambooTools.reportTestFinishedMessage();
   }
   
	@AverageDuration(duration = 26.0)
	@Test(timeout = 78091)
   public void testWalkingOverSmallPlatformQuickly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      FootstepDataList footstepDataList = createFootstepsForSteppingPastSmallPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      Point3d center = new Point3d(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      
      BambooTools.reportTestFinishedMessage();
   }

   @AverageDuration(duration = 26.0)
   @Test(timeout = 78091)
   public void testWalkingOverSmallPlatform() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

      FootstepDataList footstepDataList = createFootstepsForSteppingOntoSmallPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      if (success)
      {
         footstepDataList = createFootstepsForSteppingOffOfSmallPlatform(scriptedFootstepGenerator);
         drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      }

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(-3.7944324216932475, -5.38051322671167, 0.7893380490431007);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage();
   }

   private class TestController implements RobotController
   {
      YoVariableRegistry registry = new YoVariableRegistry("SwingHeightTestController");
      Random random = new Random();
      FullRobotModel estimatorModel;
      DoubleYoVariable maxFootHeight = new DoubleYoVariable("maxFootHeight", registry);
      DoubleYoVariable leftFootHeight = new DoubleYoVariable("leftFootHeight", registry);
      DoubleYoVariable rightFootHeight = new DoubleYoVariable("rightFootHeight", registry);
      DoubleYoVariable randomNum = new DoubleYoVariable("randomNumberInTestController", registry);
      FramePoint leftFootOrigin;
      FramePoint rightFootOrigin;

      public TestController(FullRobotModel estimatorModel){
         this.estimatorModel = estimatorModel;
      }

      @Override
      public void doControl()
      {


         ReferenceFrame leftFootFrame = estimatorModel.getFoot(RobotSide.LEFT).getBodyFixedFrame();
         leftFootOrigin = new FramePoint(leftFootFrame);
         leftFootOrigin.changeFrame(ReferenceFrame.getWorldFrame());

         ReferenceFrame rightFootFrame = estimatorModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame();
         rightFootOrigin = new FramePoint(rightFootFrame);
         rightFootOrigin.changeFrame(ReferenceFrame.getWorldFrame());

         randomNum.set(random.nextDouble());
         leftFootHeight.set(leftFootOrigin.getZ());
         rightFootHeight.set(rightFootOrigin.getZ());
         maxFootHeight.set(Math.max(maxFootHeight.getDoubleValue(), leftFootHeight.getDoubleValue()));
         maxFootHeight.set(Math.max(maxFootHeight.getDoubleValue(), rightFootHeight.getDoubleValue()));
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      public double getMaxFootHeight(){ return maxFootHeight.getDoubleValue();}
   }

   @AverageDuration
   @Test(timeout = 300000)
   public void testMultipleHeightFootsteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double[] heights = {0.1, 0.2, 0.3};
      double[] maxHeights = new double[heights.length];
      boolean success;
      for (int i = 0; i < heights.length; i++){
         double currentHeight = heights[i];
         DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
         simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
         simulationTestingParameters.setRunMultiThreaded(false);
         drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOverSmallPlatformTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());
         FullRobotModel estimatorRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
         TestController testController = new TestController(estimatorRobotModel);
         drcSimulationTestHelper.getRobot().setController(testController, 1);

         SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

         setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

         ThreadTools.sleep(1000);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0); //2.0);

         FootstepDataList footstepDataList = createBasicFootstepFromDefaultForSwingHeightTest(currentHeight);
         drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         maxHeights[i] = testController.getMaxFootHeight();
         assertTrue(success);

         if (i != heights.length - 1)
            drcSimulationTestHelper.destroySimulation();
      }

      for (int i = 0; i < heights.length -1; i++){
         if (heights[i] > heights[i+1]){
            assertTrue(maxHeights[i] > maxHeights[i+1]);
         }else{
            assertTrue(maxHeights[i] < maxHeights[i+1]);
         }
      }

      BambooTools.reportTestFinishedMessage();
   }


	@AverageDuration(duration = 16.0)
	@Test(timeout = 48040)
   public void testWalkingOntoMediumPlatformToesTouching() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.MEDIUM_PLATFORM;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoMediumPlatformToesTouchingTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingOverMediumPlatform(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForSteppingOntoMediumPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      Point3d center = new Point3d(-4.0997851961824665, -5.797669618987603, 0.9903260891750866);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage();
   }


	@AverageDuration(duration = 14.8)
	@Test(timeout = 44385)
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoMediumPlatformToesTouchingTest", "", selectedLocation,  simulationTestingParameters, getRobotModel());
   
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();
      
      setupCameraForWalkingOffOfMediumPlatform(simulationConstructionSet);
      
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      
      FootstepDataList footstepDataList = createFootstepsForSteppingOffOfMediumPlatform(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      
      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      
      assertTrue(success);
      
      Point3d center = new Point3d(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      
      BambooTools.reportTestFinishedMessage();
   }
	
   private void setupCameraForWalkingOverSmallPlatform(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(-3.0, -4.6, 0.8);
      Point3d cameraPosition = new Point3d(-11.5, -5.8, 2.5);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
   private void setupCameraForWalkingOverMediumPlatform(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(-3.9, -5.6, 0.55);
      Point3d cameraPosition = new Point3d(-7.5, -2.3, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
   private void setupCameraForWalkingOffOfMediumPlatform(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(-3.9, -5.6, 0.55);
      Point3d cameraPosition = new Point3d(-7.6, -2.4, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private FootstepDataList createFootstepsForSteppingOntoSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.3303508964136372, -5.093152916934431, 0.2361869051765919}, {-0.003380023644676521, 0.01519186055257256, 0.9239435001894032, -0.3822122332825927}},
            {{-3.4980005080184333, -4.927710662235891, 0.23514263035532196}, {-6.366244432153206E-4, -2.2280928201561157E-4, 0.9240709626189128, -0.3822203567445069}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForSteppingOffOfSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.850667406347062, -5.249955436839419, 0.08402883817600326}, {-0.0036296745847858064, 0.003867481752280881, 0.9236352342329301, -0.38323598752046323}},
            {{-3.6725725349280296, -5.446807690769805, 0.08552806597763604}, {-6.456929194763128E-5, -0.01561897825296648, 0.9234986484659182, -0.3832835629540643}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForSteppingPastSmallPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-3.3303508964136372, -5.093152916934431, 0.2361869051765919}, {-0.003380023644676521, 0.01519186055257256, 0.9239435001894032, -0.3822122332825927}},
            {{-3.850667406347062, -5.249955436839419, 0.08402883817600326}, {-0.0036296745847858064, 0.003867481752280881, 0.9236352342329301, -0.38323598752046323}},
            {{-3.6725725349280296, -5.446807690769805, 0.08552806597763604}, {-6.456929194763128E-5, -0.01561897825296648, 0.9234986484659182, -0.3832835629540643}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      FootstepDataList desiredFootsteps = scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
      double zClearHeight = desiredFootsteps.get(0).getLocation().getZ() + 0.07;
      double swingHeightForClear = zClearHeight - desiredFootsteps.get(2).getLocation().getZ(); //should really be the last height (height before swing), not step 2, but they're approximate.
      desiredFootsteps.get(1).setSwingHeight(swingHeightForClear);
      desiredFootsteps.get(1).setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE);
      return desiredFootsteps;
   }

   private FootstepDataList createBasicFootstepFromDefaultForSwingHeightTest(double swingHeight)
   {
      FootstepDataList desiredFootsteps =  new FootstepDataList(0.0, 0.0);
      FootstepData footstep = new FootstepData(RobotSide.RIGHT, new Point3d(0.4, -0.125, 0.085), new Quat4d(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE);
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.footstepDataList.add(footstep);
      return desiredFootsteps;
   }
   
   
   private FootstepDataList createFootstepsForSteppingOntoMediumPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-4.144889177599215, -5.68009276450442, 0.2841471307289875}, {-0.012979910123161926, 0.017759854548746876, 0.9232071519598507, -0.3836726001029824}},
            {{-3.997325285359919, -5.8527640256176685, 0.2926905844610473}, {-0.022159348866436335, -0.014031420240348416, 0.9230263369316307, -0.3838417171627259}}
            };
      
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
   
   private FootstepDataList createFootstepsForSteppingOffOfMediumPlatform(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{-4.304392715667327, -6.084498586699763, 0.08716704456087025}, {-0.0042976203878775715, -0.010722204803598987, 0.9248070170408506, -0.38026115501738456}},
            {{-4.4394706079327255, -5.9465856725464565, 0.08586305720146342}, {-8.975861226689934E-4, 0.002016837110644428, 0.9248918980282926, -0.380223754740342}},
            };
      
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
 

}
