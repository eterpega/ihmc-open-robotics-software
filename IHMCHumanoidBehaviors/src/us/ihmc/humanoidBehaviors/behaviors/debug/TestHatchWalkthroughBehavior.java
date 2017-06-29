package us.ihmc.humanoidBehaviors.behaviors.debug;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
//import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.Hatch;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestHatchWalkthroughBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyTransform hatchToWorldTransform;
   private final ReferenceFrame hatchFrame;
   
   private final HumanoidReferenceFrames referenceFrames;
   private final YoDouble swingTime = new YoDouble("BehaviorSwingTime", registry);
   private final YoDouble sleepTime = new YoDouble("BehaviorSleepTime", registry);
   private final YoDouble transferTime = new YoDouble("BehaviorTransferTime", registry);
   private final YoDouble stepLength = new YoDouble("BehaviorStepLength", registry);
   private final YoBoolean stepInPlace = new YoBoolean("StepInPlace", registry);
   private final YoBoolean abortBehavior = new YoBoolean("AbortBehavior", registry);

   private final YoStopwatch timer;
   
   private Hatch hatch = HatchEnvironment.getHatch(0);
   double hatchWidth = hatch.getWidth();
   double hatchThickness = hatch.getThickness();
   double hatchLowerHeight = hatch.getStepHeight();
   double hatchUpperHeight = hatch.getOpeningHeight();
   
   private final Point3D defaultFootSwingWayPoint1 = new Point3D(0.03, 0.00, 0.10);
   private final Point3D defaultFootSwingWayPoint2 = new Point3D(-0.03, 0.00, 0.08);
   private Point3D leftFootSwingWayPoint1 = new Point3D();
   private Point3D leftFootSwingWayPoint2 = new Point3D();
   private Point3D rightFootSwingWayPoint1 = new Point3D();
   private Point3D rightFootSwingWayPoint2 = new Point3D();
   
   private Point3D rightFootSwingGoalPoint = new Point3D();
   private Point3D rightBeforeHatchOffset = new Point3D();
   private Point3D rightAfterHatchOffset = new Point3D();
   
   private Point3D leftFootSwingGoalPoint = new Point3D();
   private Point3D leftBeforeHatchOffset = new Point3D();
   private Point3D leftAfterHatchOffset = new Point3D();
   
   private double[] pelvisRollPitchYawInitialization = new double[3];
   private double[] pelvisRollPitchYawFirstStepThroughHatch = new double[3];
   private double[] pelvisRollPitchYawReconfiguration = new double[3];
   private double[] pelvisRollPitchYawSecondStepThroughHatch = new double[3];
   
   private Vector3D pelvisMovementInitialization = new Vector3D();
   private Vector3D pelvisMovementFirstStepThroughHatch = new Vector3D();
   private Vector3D pelvisMovementReconfiguration = new Vector3D();
   private Vector3D pelvisMovementSecondStepThroughHatch = new Vector3D();
   
   double armTrajectoryTime; // was 4.0
   double leftArmGoalPosition[] = new double[] { -1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0 };
   double rightArmGoalPosition[] = new double[] { 1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0 };
   boolean madeFirstStepThroughHatch = false;
   boolean madeSecondStepThroughHatch = false;
   boolean robotConfigurationInitialized = false;
   boolean changedRobotConfiguration = false;
   boolean finalState = false;
   double counterHelper = 0;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(10);

   private int version = 2;

   public TestHatchWalkthroughBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, YoDouble yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;

      swingTime.set(1.2);
      transferTime.set(0.6);
      sleepTime.set(10.0);
      stepLength.set(0.3);

      timer = new YoStopwatch(yoTime);
      
      switch (version)
      {
      case 0:
         armTrajectoryTime = 4.0;
         break;
      case 1:
         armTrajectoryTime = 2.0;
         break;
      case 2:
         armTrajectoryTime = 1.0;
         break;
      default:
         break;
      }
      
      hatchToWorldTransform = hatch.getHatchToWorldTransform();
      hatchToWorldTransform.invert();
      hatchFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("HatchFrame", worldFrame, hatchToWorldTransform);
      
//      PrintTools.debug("Transform = " + hatchToWorldTransform.toString());
      
      rightBeforeHatchOffset.set(hatchToWorldTransform.getTranslationX(), 0.0, 0.0);
      rightAfterHatchOffset.set(rightBeforeHatchOffset.getX() - 0.01, 0.03, 0.0);
      
      leftBeforeHatchOffset.set(hatchToWorldTransform.getTranslationX(), 0.0, 0.0);
      leftAfterHatchOffset.set(leftBeforeHatchOffset.getX() + 0.03 - 0.01, 0.03, 0.0);
      
      setFootSwingGoalPointsBasedOnHatchDimensions();
      setFootSwingWayPointsBasedOnHatchDimensions();
      setPelvisTrajectoriesBasedOnHatchDimensions();
   }
   
   @Override
   public void doControl()
   {
      // TODO: Fix all chest trajectories!!!
      // Use version without ducking
//      walkThroughHatchIteration1();
      
      // Use version with ducking
//      walkThroughHatchIteration2();
      
      // Use version with whole body trajectories
//      walkThroughHatchWBT();
      
      // Use foot swing tunnel version
//      walkThroughHatchTunnel();
      
      // Test for pelvis coordination
      walkThroughHatchCoordinated();
      
   // Test for pelvis coordination OLD VERSION
//      walkThroughHatchCoordinatedOld();
   }
   
   public void walkThroughHatchCoordinated()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfigurationCoordinated();
      }

      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
      {
         return;
      }
      
      if(!madeFirstStepThroughHatch)
      {  
         makeFirstStepThroughHatchOpeningCoordinated();
         counterHelper = timer.totalElapsed();
      }
      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 3.5 * armTrajectoryTime)))
      {
         changeRobotConfigurationCoordinated();
         counterHelper = timer.totalElapsed();
      }
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 3.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpeningCoordinated();
         counterHelper = timer.totalElapsed();
      }
      else if(!finalState && (timer.totalElapsed() > (counterHelper + 3.5 * armTrajectoryTime)))
      {
         finalState();
      }
   }
   
   public void initializeRobotConfigurationCoordinated()
   {
      ReferenceFrame chestFrame = referenceFrames.getChestFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(chestFrame, 0.0, Math.toRadians(17.0), 0.0);
      Quaternion chestOrientationLocal = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationLocal);
//      PrintTools.debug(this, chestOrientationLocal.toString());
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationLocal, chestFrame, referenceFrames.getPelvisZUpFrame());
      
      
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawInitialization[0], pelvisRollPitchYawInitialization[1], pelvisRollPitchYawInitialization[2]);
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
      
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientationWorld);
      
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setPosition(pelvisMovementInitialization);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      sendPacket(chestOrientationTrajectoryMessage);
      sendPacket(pelvisOrientationTrajectoryMessage);
      sendPacket(leftArmTrajectoryMessage);
      sendPacket(rightArmTrajectoryMessage);

      robotConfigurationInitialized = true;
      
//      // +++++ DEBUG +++++
      FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
//      pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisGoalLocation.setToZero();
      pelvisPose2.getPosition(pelvisGoalLocation);
      PrintTools.debug(this, "intialize pelvis 1 = " + pelvisGoalLocation.toString());
      
      pelvisPose2.changeFrame(hatchFrame);
      pelvisGoalLocation.setToZero();
      pelvisPose2.getPosition(pelvisGoalLocation);
      PrintTools.debug(this, "intialize pelvis 2 = " + pelvisGoalLocation.toString());
//      chestFrame = referenceFrames.getChestFrame();
//      chestOrientationFrame = new FrameOrientation(chestFrame, 0.0, Math.toRadians(17.0), 0.0);
//      chestOrientationLocal = new Quaternion();
//      chestOrientationFrame.getQuaternion(chestOrientationLocal);
//      PrintTools.debug(this, chestOrientationLocal.toString());
//      
//      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0)); //TEST: was -15.0
//      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
//      PrintTools.debug(this, pelvisOrientation.toString());
//      
//      pelvisFrame = referenceFrames.getPelvisFrame();
//      pelvisPose = new FramePose(pelvisFrame);
//      pelvisPose.setZ(-0.08); // -.065
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
//      // ++++++ END ++++++
   }
   
   public void makeFirstStepThroughHatchOpeningCoordinated()
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-7.0), Math.toRadians(27.0), Math.toRadians(0.0));
      Quaternion chestOrientationPelvisZUp = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
//      PrintTools.debug(this, chestOrientationPelvisZUp.toString());
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5*armTrajectoryTime, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
      

      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawFirstStepThroughHatch[0], pelvisRollPitchYawFirstStepThroughHatch[1], pelvisRollPitchYawFirstStepThroughHatch[2]);
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
      
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.5*armTrajectoryTime, pelvisOrientationWorld); // was 1.5
      
      
      
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.setPosition(pelvisMovementFirstStepThroughHatch);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(3.0*armTrajectoryTime, pelvisGoalLocation.getZ());
      
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setPosition(rightFootSwingGoalPoint);
      
      FramePose wayPointPose1 = new FramePose(rightSoleFrame);
      wayPointPose1.setX(rightFootSwingWayPoint1.getX());
      wayPointPose1.setY(rightFootSwingWayPoint1.getY());
      wayPointPose1.setZ(rightFootSwingWayPoint1.getZ());
      FramePose wayPointPose2 = new FramePose(rightSoleFrame);
      wayPointPose2.setX(stepPose.getX() + rightFootSwingWayPoint2.getX());
      wayPointPose2.setY(stepPose.getY() + rightFootSwingWayPoint2.getY());
      wayPointPose2.setZ(stepPose.getZ() + rightFootSwingWayPoint2.getZ());

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      

      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);

      
      // send trajectory packets
      sendPacket(chestOrientationTrajectoryMessage);
      sendPacket(pelvisOrientationTrajectoryMessage); 
      sendPacket(pelvisHeightTrajectoryMessage);
      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
      
//      // +++++ DEBUG +++++
      FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
//    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "first step pelvis 1 = " + pelvisGoalLocation.toString());
    
    pelvisPose2.changeFrame(hatchFrame);
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "first step pelvis 2 = " + pelvisGoalLocation.toString());
//      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
//      chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-7.0), Math.toRadians(25.0), Math.toRadians(0.0));
//      chestOrientationPelvisZUp = new Quaternion();
//      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
//      PrintTools.debug(this, chestOrientationPelvisZUp.toString());
//      
//
//      pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(7.0), Math.toRadians(5.0), Math.toRadians(-7.0));
//      pelvisOrientationWorld = new Quaternion();
//      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
//
//      
//      pelvisPose = new FramePose(pelvisZUpFrame);
//      pelvisPose.setX(0.05); // 0.0
//      pelvisPose.setZ(0.02); //0.3
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
//      // ++++++ END ++++++
   }
   
   public void changeRobotConfigurationCoordinated()
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(25.0), Math.toRadians(0.0)); //25?
      Quaternion chestOrientationPelvisZUp = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
//      PrintTools.debug(this, chestOrientationPelvisZUp.toString());
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5*armTrajectoryTime, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame); //2.5
      sendPacket(chestOrientationTrajectoryMessage);
      
      
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawReconfiguration[0], pelvisRollPitchYawReconfiguration[1], pelvisRollPitchYawReconfiguration[2]);
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
      
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.setPosition(pelvisMovementReconfiguration);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
      
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(1.5*armTrajectoryTime, pelvisGoalLocation, pelvisOrientationWorld);
      
      
      // send trajectory packets
      sendPacket(pelvisTrajectoryMessage);
      changedRobotConfiguration = true;
      
//      // +++++ DEBUG +++++
      FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
//    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "transition pelvis 1 = " + pelvisGoalLocation.toString());
    
    pelvisPose2.changeFrame(hatchFrame);
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "transition pelvis 2 = " + pelvisGoalLocation.toString());
//      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
//      chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(20.0), Math.toRadians(0.0));
//      chestOrientationPelvisZUp = new Quaternion();
//      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
//      PrintTools.debug(this, chestOrientationPelvisZUp.toString());
//
//      
//      pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(10.0), Math.toRadians(0.0)); // roll was 15.0
//      pelvisOrientationWorld = new Quaternion();
//      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
//      
//      
//      pelvisPose = new FramePose(pelvisZUpFrame);
//      pelvisPose.setX(pelvisPose.getX() + 0.10); // TEST: was 0.15
//      pelvisPose.setZ(pelvisPose.getZ() + 0.02); // TEST: was 0.02
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
//      // ++++++ END ++++++
   }
   
   public void makeSecondStepThroughHatchOpeningCoordinated()
   {     
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 7.0
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame()); // was 2.0
//      PrintTools.debug(this, chestOrientation.toString());
      
      
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawSecondStepThroughHatch[0], pelvisRollPitchYawSecondStepThroughHatch[1], pelvisRollPitchYawSecondStepThroughHatch[2]);
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.0*armTrajectoryTime, pelvisOrientationWorld); // was 1.5
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
      
      
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setPosition(leftFootSwingGoalPoint);
      FramePose wayPointPose1 = new FramePose(leftSoleFrame);
      wayPointPose1.setX(leftFootSwingWayPoint1.getX());
      wayPointPose1.setY(leftFootSwingWayPoint1.getY());
      wayPointPose1.setZ(leftFootSwingWayPoint1.getZ());
      FramePose wayPointPose2 = new FramePose(leftSoleFrame);
      wayPointPose2.setX(stepPose.getX() + leftFootSwingWayPoint2.getX());
      wayPointPose2.setY(stepPose.getY() + leftFootSwingWayPoint2.getY());
      wayPointPose2.setZ(stepPose.getZ() + leftFootSwingWayPoint2.getZ());

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);
      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);
      
      
      // send trajectory packets
      sendPacket(chestOrientationTrajectoryMessage);
      sendPacket(pelvisOrientationTrajectoryMessage);
      sendPacket(pelvisHeightTrajectoryMessage);
      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
      
//      // +++++ DEBUG +++++
      FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
//    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "second step pelvis 1 = " + pelvisGoalLocation.toString());
    
    pelvisPose2.changeFrame(hatchFrame);
    pelvisGoalLocation.setToZero();
    pelvisPose2.getPosition(pelvisGoalLocation);
    PrintTools.debug(this, "second step pelvis 2 = " + pelvisGoalLocation.toString());
//      chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 7.0
//      chestOrientation = new Quaternion(chestOrientationAA);
//      PrintTools.debug(this, chestOrientation.toString());
//
//      
//      pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-5.0), Math.toRadians(15.0), Math.toRadians(10.0));
//      pelvisOrientationWorld = new Quaternion();
//      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
//      PrintTools.debug(this, pelvisOrientationWorld.toString());
//      
//      
//      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
//      pelvisPose = new FramePose(pelvisZUpFrame);
////      pelvisPose.setZ(0.02);
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//      PrintTools.debug(this, pelvisGoalLocation.toString());
//      // ++++++ END ++++++
   }
   
   public void finalState()
   {     
      finalState = true;
         
      // +++++ DEBUG +++++
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      PrintTools.debug(this, "second step pelvis 1 = " + pelvisPose.getPosition().toString());
    
      pelvisPose.changeFrame(hatchFrame);
      PrintTools.debug(this, "second step pelvis 2 = " + pelvisPose.getPosition().toString());
      // ++++++ END ++++++
   }
   
   
   private void setChestTrajectoryBasedOnHatchDimensions()
   {
      
   }
   
   private void setPelvisTrajectoriesBasedOnHatchDimensions()
   {
      pelvisRollPitchYawInitialization[0] = Math.toRadians(0.0);
      pelvisRollPitchYawInitialization[1] = Math.toRadians(-hatchLowerHeight*100.0); //TEST: was -15.0 for 0.15 height
      pelvisRollPitchYawInitialization[2] = Math.toRadians(0.0);
      
      double pelvisMovementInitializationZ = -0.215 + 0.0475 * hatchLowerHeight/0.05 + (hatchUpperHeight - 1.55);
      if(pelvisMovementInitializationZ > 0.0)
         pelvisMovementInitializationZ = 0.0;
      pelvisMovementInitialization.set(0.0, 0.0, pelvisMovementInitializationZ);
      
      pelvisRollPitchYawFirstStepThroughHatch[0] = Math.toRadians(7.0);
      pelvisRollPitchYawFirstStepThroughHatch[1] = Math.toRadians(5.0);
      pelvisRollPitchYawFirstStepThroughHatch[2] = Math.toRadians(-7.0); // -7.0
      
      pelvisMovementFirstStepThroughHatch.set(0.05, 0.0, 0.02);
      
      pelvisRollPitchYawReconfiguration[0] = Math.toRadians(0.0);
      pelvisRollPitchYawReconfiguration[1] = Math.toRadians(4.0 + 2.0 * hatchLowerHeight/0.05);
      pelvisRollPitchYawReconfiguration[2] = Math.toRadians(0.0); // roll was 15.0 for 0.15 height

//      pelvisMovementReconfiguration.set(-0.05 + hatchThickness, 0.0, 0.02);
      pelvisMovementReconfiguration.set(0.025 + 0.025*hatchLowerHeight/0.05, 0.0, 0.02);
   
      pelvisRollPitchYawSecondStepThroughHatch[0] = Math.toRadians(-5.0);
      pelvisRollPitchYawSecondStepThroughHatch[1] = Math.toRadians(hatchLowerHeight*100.0);
      pelvisRollPitchYawSecondStepThroughHatch[2] = Math.toRadians(10.0);
      
      pelvisMovementSecondStepThroughHatch.set(0.0, 0.0, 0.0);
      
   }
   
   private void setFootSwingGoalPointsBasedOnHatchDimensions()
   {
      setRightFootSwingGoalPointBasedOnHatchDimensions();
      setLeftFootSwingGoalPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingGoalPointBasedOnHatchDimensions()
   {
      rightFootSwingGoalPoint.add(rightBeforeHatchOffset, rightAfterHatchOffset);
      rightFootSwingGoalPoint.add(new Point3D(hatchThickness, 0.0, 0.0));
   }
   
   private void setLeftFootSwingGoalPointBasedOnHatchDimensions()
   {
      leftFootSwingGoalPoint.add(leftBeforeHatchOffset, leftAfterHatchOffset);
      leftFootSwingGoalPoint.add(new Point3D(hatchThickness, 0.0, 0.0));
   }
   
   private void setFootSwingWayPointsBasedOnHatchDimensions()
   {
      setRightFootSwingWayPointBasedOnHatchDimensions();
      setLeftFootSwingWayPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingWayPointBasedOnHatchDimensions()
   {
      rightFootSwingWayPoint1.add(defaultFootSwingWayPoint1, new Point3D(0, -0.03, hatchLowerHeight));
      rightFootSwingWayPoint2.add(defaultFootSwingWayPoint2, new Point3D(0, -0.03, hatchLowerHeight));
   }
   
   private void setLeftFootSwingWayPointBasedOnHatchDimensions()
   {
      leftFootSwingWayPoint1.add(defaultFootSwingWayPoint1, new Point3D(0, 0.03, hatchLowerHeight - 0.02));
      leftFootSwingWayPoint2.add(defaultFootSwingWayPoint2, new Point3D(0, 0.03, hatchLowerHeight));
   }
   
   
   
   // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   
   
   
   
   public void walkThroughHatchCoordinatedOld()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfigurationCoordinatedOld();
      }

      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
      {
         return;
      }
      
      if(!madeFirstStepThroughHatch)
      {  
         makeFirstStepThroughHatchOpeningCoordinatedOld();
         counterHelper = timer.totalElapsed();
      }
      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 3.5 * armTrajectoryTime)))
      {
         changeRobotConfigurationCoordinatedOld();
         counterHelper = timer.totalElapsed();
      }
      
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 3.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpeningCoordinatedOld();
      }
   }
   
   public void initializeRobotConfigurationCoordinatedOld()
   {
      ReferenceFrame chestFrame = referenceFrames.getChestFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(chestFrame, 0.0, Math.toRadians(17.0), 0.0);
      Quaternion chestOrientationLocal = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationLocal);
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationLocal, chestFrame, referenceFrames.getPelvisZUpFrame());
      sendPacket(chestOrientationTrajectoryMessage);
      
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0)); //TEST: was -15.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(-0.08); // -.065
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      sendPacket(leftArmTrajectoryMessage);
      sendPacket(rightArmTrajectoryMessage);

      robotConfigurationInitialized = true;
   }
   
   public void makeFirstStepThroughHatchOpeningCoordinatedOld()
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-7.0), Math.toRadians(25.0), Math.toRadians(0.0));
      Quaternion chestOrientationPelvisZUp = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5*armTrajectoryTime, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
      sendPacket(chestOrientationTrajectoryMessage);

      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(7.0), Math.toRadians(5.0), Math.toRadians(-7.0));
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
      
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.5*armTrajectoryTime, pelvisOrientationWorld); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.setX(0.05); // 0.0
      pelvisPose.setZ(0.02); //0.3
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(3.0*armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setX(0.60);
      stepPose.setY(0.03);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
      FramePose wayPointPose1 = new FramePose(rightSoleFrame);
      wayPointPose1.setX(0.03);
      wayPointPose1.setY(-0.03);
      wayPointPose1.setZ(0.25);
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      
      FramePose wayPointPose2 = new FramePose(rightSoleFrame);
      wayPointPose2.setX(0.53);
      wayPointPose2.setY(0.00);
      wayPointPose2.setZ(0.24);
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);


      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfigurationCoordinatedOld()
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(20.0), Math.toRadians(0.0));
      Quaternion chestOrientationPelvisZUp = new Quaternion();
      chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
      
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5*armTrajectoryTime, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
      sendPacket(chestOrientationTrajectoryMessage);
      
      
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
      pelvisPose.setX(pelvisPose.getX() + 0.10); // TEST: was 0.15
      pelvisPose.setZ(pelvisPose.getZ() + 0.02); // TEST: was 0.02

      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(10.0), Math.toRadians(0.0)); // roll was 15.0
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
      
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(1.5*armTrajectoryTime, pelvisGoalLocation, pelvisOrientationWorld);
      sendPacket(pelvisTrajectoryMessage);
      
      changedRobotConfiguration = true;
   }
   
   public void makeSecondStepThroughHatchOpeningCoordinatedOld()
   {     
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 7.0
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame()); // was 2.0
      sendPacket(chestOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FramePose pelvisPose = new FramePose(pelvisZUpFrame);
//      pelvisPose.setZ(0.02);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
   
      
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-5.0), Math.toRadians(15.0), Math.toRadians(10.0));
      Quaternion pelvisOrientationWorld = new Quaternion();
      pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
      
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.0*armTrajectoryTime, pelvisOrientationWorld); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.63);
      stepPose.setY(0.03);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
      FramePose wayPointPose1 = new FramePose(leftSoleFrame);
      wayPointPose1.setX(0.03);
      wayPointPose1.setY(0.06);
      wayPointPose1.setZ(0.25);
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      
      FramePose wayPointPose2 = new FramePose(leftSoleFrame);
      wayPointPose2.setX(0.60);
      wayPointPose2.setY(0.03);
      wayPointPose2.setZ(0.24);
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);
      

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   
   
   
   
   
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
   
   
   
   public void walkThroughHatchTunnel()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfigurationTunnel();
      }

      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
      {
         return;
      }
      
      if(!madeFirstStepThroughHatch)
      {  
         makeFirstStepThroughHatchOpeningTunnel();
         counterHelper = timer.totalElapsed();
      }
      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         changeRobotConfigurationTunnel();
         counterHelper = timer.totalElapsed();
      }
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpeningTunnel();
      }
   }
   
   
   public void initializeRobotConfigurationTunnel()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(15.0));
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);
      
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0)); //TEST: was -15.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(-0.06); // -.04
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      sendPacket(leftArmTrajectoryMessage);
      sendPacket(rightArmTrajectoryMessage);

      robotConfigurationInitialized = true;
   }
   
   public void makeFirstStepThroughHatchOpeningTunnel()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(30.0)); // was 25
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(4.5*armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);

      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); // TEST: was -5.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(4.5*armTrajectoryTime, pelvisOrientation); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.08);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.25);

      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfigurationTunnel()
   {
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(20.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setX(0.15); // TEST: was 0.15
      pelvisPose.setZ(-0.05); // TEST: was -0.07

      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation, pelvisGoalOrientation);
      sendPacket(pelvisTrajectoryMessage);
      
      changedRobotConfiguration = true;
   }
   
   public void makeSecondStepThroughHatchOpeningTunnel()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 10.0
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame()); // was 2.0
      sendPacket(chestOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(0.02); // TEST: was 0.04 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
   
      
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(20.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(3.0)); // TEST: was 3.0
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisGoalOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.00);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.35);

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   
   
   
   // ===========================================================================================================
   
   public void walkThroughHatchWBT()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfigurationWBT();
      }

//      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
//      {
//         return;
//      }
//      
//      if(!madeFirstStepThroughHatch)
//      {  
//         makeFirstStepThroughHatchOpeningWBT();
//         counterHelper = timer.totalElapsed();
//      }
//      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
//      {
//         changeRobotConfigurationWBT();
//         counterHelper = timer.totalElapsed();
//      }
//      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
//      {
//         makeSecondStepThroughHatchOpeningWBT();
//      }
   }
   
   
   public void initializeRobotConfigurationWBT()
   {
//      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
//      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
//      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation);
//      sendPacket(chestOrientationTrajectoryMessage);
//      
//      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0)); //TEST: was -15.0
//      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
//      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientation);
//      sendPacket(pelvisOrientationTrajectoryMessage);
//      
//      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
//      FramePose pelvisPose = new FramePose(pelvisFrame);
//      pelvisPose.setZ(-0.05); // -.04
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      Point3D pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//
//      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
//      sendPacket(pelvisHeightTrajectoryMessage);
      
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(4);
      
      double trajectoryTime = 1.0;
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      
      AxisAngle pelvisOrientationWaypoint0AA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisOrientationWaypoint0 = new Quaternion(pelvisOrientationWaypoint0AA);
      pelvisTrajectoryMessage.setTrajectoryPoint(0, 1.0*trajectoryTime, pelvisGoalLocation, pelvisOrientationWaypoint0, new Vector3D(), new Vector3D());
 
      AxisAngle pelvisOrientationWaypoint1AA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisOrientationWaypoint1 = new Quaternion(pelvisOrientationWaypoint1AA);
      pelvisGoalLocation.setZ(pelvisGoalLocation.getZ()-0.5);
      pelvisTrajectoryMessage.setTrajectoryPoint(1, 2.0*trajectoryTime, pelvisGoalLocation, pelvisOrientationWaypoint1, new Vector3D(), new Vector3D());

      AxisAngle pelvisOrientationWaypoint2AA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisOrientationWaypoint2 = new Quaternion(pelvisOrientationWaypoint2AA);
      pelvisTrajectoryMessage.setTrajectoryPoint(2, 3.0*trajectoryTime, pelvisGoalLocation, pelvisOrientationWaypoint2, new Vector3D(), new Vector3D());

      AxisAngle pelvisOrientationWaypoint3AA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisOrientationWaypoint3 = new Quaternion(pelvisOrientationWaypoint3AA);
      //pelvisGoalLocation.setZ(pelvisGoalLocation.getZ()+0.10);
      pelvisTrajectoryMessage.setTrajectoryPoint(3, 4.0*trajectoryTime, pelvisGoalLocation, pelvisOrientationWaypoint3, new Vector3D(), new Vector3D());

      
      FootTrajectoryMessage rightFootTrajectoryMessage = new FootTrajectoryMessage(RobotSide.RIGHT, 4);
      
      
      wholeBodyTrajectoryMessage.leftArmTrajectoryMessage = leftArmTrajectoryMessage;
      wholeBodyTrajectoryMessage.rightArmTrajectoryMessage = rightArmTrajectoryMessage;
      wholeBodyTrajectoryMessage.pelvisTrajectoryMessage = pelvisTrajectoryMessage;
      
      sendPacket(wholeBodyTrajectoryMessage);
      robotConfigurationInitialized = true;
   }
   
   public void makeFirstStepThroughHatchOpeningWBT()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(25.0)); // was 25
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(4.5*armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);

      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); // TEST: was -5.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(4.5*armTrajectoryTime, pelvisOrientation); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.08);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.17);

      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfigurationWBT()
   {
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setX(0.15); // TEST: was 15.0
      pelvisPose.setZ(-8.0); // TEST: was -7.0

      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation, pelvisGoalOrientation);
      sendPacket(pelvisTrajectoryMessage);
      
      changedRobotConfiguration = true;
   }
   
   public void makeSecondStepThroughHatchOpeningWBT()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 10.0
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame()); // was 2.0
      sendPacket(chestOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(0.04); // TEST: was 0.03
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
   
      
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(3.0)); // TEST: was 3.0
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisGoalOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.00);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
      FramePose wayPointPose1 = new FramePose(leftSoleFrame);
      wayPointPose1.setX(0.06);
      wayPointPose1.setY(-0.06);
      wayPointPose1.setZ(0.25);
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      
      FramePose wayPointPose2 = new FramePose(leftSoleFrame);
      wayPointPose2.setX(0.45);
      wayPointPose2.setY(0.00);
      wayPointPose2.setZ(0.19);
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   
   
   
   
   
   // ====================================================================================================
   
   
   
   
   
   
   public void walkThroughHatchIteration2()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfigurationIteration2();
      }

      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
      {
         return;
      }
      
      if(!madeFirstStepThroughHatch)
      {  
         makeFirstStepThroughHatchOpeningIteration2();
         counterHelper = timer.totalElapsed();
      }
      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         changeRobotConfigurationIteration2();
         counterHelper = timer.totalElapsed();
      }
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpeningIteration2();
      }
   }
   
   
   public void initializeRobotConfigurationIteration2()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);
      
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0)); //TEST: was -15.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(-0.05); // -.04
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      sendPacket(leftArmTrajectoryMessage);
      sendPacket(rightArmTrajectoryMessage);

      robotConfigurationInitialized = true;
   }
   
   public void makeFirstStepThroughHatchOpeningIteration2()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(25.0)); // was 25
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(4.5*armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);

      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); // TEST: was -5.0
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);

      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(4.5*armTrajectoryTime, pelvisOrientation); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.08);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.17);

      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfigurationIteration2()
   {
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setX(0.15); // TEST: was 15.0
      pelvisPose.setZ(-8.0); // TEST: was -7.0

      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation, pelvisGoalOrientation);
      sendPacket(pelvisTrajectoryMessage);
      
      changedRobotConfiguration = true;
   }
   
   public void makeSecondStepThroughHatchOpeningIteration2()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(7.0)); // was 10.0
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame()); // was 2.0
      sendPacket(chestOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(0.04); // TEST: was 0.03
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
   
      
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(3.0)); // TEST: was 3.0
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisGoalOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.00);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
      FramePose wayPointPose1 = new FramePose(leftSoleFrame);
      wayPointPose1.setX(0.06);
      wayPointPose1.setY(-0.06);
      wayPointPose1.setZ(0.25);
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      
      FramePose wayPointPose2 = new FramePose(leftSoleFrame);
      wayPointPose2.setX(0.45);
      wayPointPose2.setY(0.00);
      wayPointPose2.setZ(0.19);
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   
   
   public void walkThroughHatchIteration1()
   {
      if (!robotConfigurationInitialized)
      {
         initializeRobotConfiguration();
      }

      if (!(timer.totalElapsed() > sleepTime.getDoubleValue()))
      {
         return;
      }
      
      if(!madeFirstStepThroughHatch)
      {  
         makeFirstStepThroughHatchOpening();
         counterHelper = timer.totalElapsed();
      }
      else if(!changedRobotConfiguration && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         changeRobotConfiguration();
         counterHelper = timer.totalElapsed();
      }
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpening();
      }
   }
   
   public void initializeRobotConfiguration()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      sendPacket(chestOrientationTrajectoryMessage);
      
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0));
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(armTrajectoryTime, pelvisOrientation);
      sendPacket(pelvisOrientationTrajectoryMessage);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setZ(-0.04);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
      sendPacket(pelvisHeightTrajectoryMessage);
      
      ArmTrajectoryMessage leftArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.LEFT, armTrajectoryTime, leftArmGoalPosition);
      ArmTrajectoryMessage rightArmTrajectoryMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, armTrajectoryTime, rightArmGoalPosition);
      
      sendPacket(leftArmTrajectoryMessage);
      sendPacket(rightArmTrajectoryMessage);

      robotConfigurationInitialized = true;
   }
   
   public void makeFirstStepThroughHatchOpening()
   {
      //Do this when restructuring code
//      FramePoint startPointInFrontOfHatch = new FramePoint(hatchFrame, -0.1, 0.0, 0.0);
//      startPointInFrontOfHatch.changeFrame(worldFrame);
      
      // Not present for 4.0 time
      if(version == 1 || version == 2)
      {
         AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(25.0)); // was 10.0
         Quaternion chestOrientation = new Quaternion(chestOrientationAA);
         ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
         sendPacket(chestOrientationTrajectoryMessage);
      }
      
      
      
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-5.0)); // was now 0!!! //was 10.0, (might have) worked
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      double factor = 1.5;
      if(version == 1)
      {
         factor = 2.5;
      }
      else if(version == 2)
      {
         factor = 4.5;
      }
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(factor*armTrajectoryTime, pelvisOrientation); // was 1.5
      sendPacket(pelvisOrientationTrajectoryMessage);
      
//      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
//      FramePose pelvisPose = new FramePose(pelvisFrame);
//      pelvisPose.setZ(0.04);
//      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//      Point3D pelvisGoalLocation = new Point3D();
//      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
//
//      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.5*armTrajectoryTime, pelvisGoalLocation.getZ());
//      sendPacket(pelvisHeightTrajectoryMessage);
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame rightSoleFrame = referenceFrames.getSoleFrame(RobotSide.RIGHT);
      FramePose stepPose = new FramePose(rightSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.08);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.RIGHT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.17);

      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfiguration()
   {
      AxisAngle pelvisGoalOrientationYAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion pelvisGoalOrientationY = new Quaternion(pelvisGoalOrientationYAA);
      AxisAngle pelvisGoalOrientationXAA = new AxisAngle(1.0, 0.0, 0.0, Math.toRadians(0.0));
      Quaternion pelvisGoalOrientationX = new Quaternion(pelvisGoalOrientationXAA);
      
      Quaternion pelvisGoalOrientation = new Quaternion();
      pelvisGoalOrientation.multiply(pelvisGoalOrientationX, pelvisGoalOrientationY);
      
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      FramePose pelvisPose = new FramePose(pelvisFrame);
      pelvisPose.setX(0.15); // was 0.12 before test
      pelvisPose.setZ(-7.0); // Added as test! 5
      if(version == 0)
      {
       pelvisPose.setZ(0.05); // present for 4.0
      }
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D pelvisGoalLocation = new Point3D();
      
      pelvisPose.getPose(pelvisGoalLocation, new Quaternion());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation, pelvisGoalOrientation);
      sendPacket(pelvisTrajectoryMessage);
      
      changedRobotConfiguration = true;
   }
   
   public void makeSecondStepThroughHatchOpening()
   {
      // Not present for 4.0
      if(version == 1 || version == 2)
      {
         ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
         FramePose pelvisPose = new FramePose(pelvisFrame);
         pelvisPose.setZ(0.05);
         pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D pelvisGoalLocation = new Point3D();
         pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
         PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(armTrajectoryTime, pelvisGoalLocation.getZ());
         sendPacket(pelvisHeightTrajectoryMessage);
      }
      
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(0.00);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
//      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
      FramePose wayPointPose1 = new FramePose(leftSoleFrame);
      wayPointPose1.setX(0.06);
      wayPointPose1.setY(-0.06);
      wayPointPose1.setZ(0.25);
      wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint1 = new Point3D();
      wayPointPose1.getPose(locationWayPoint1, new Quaternion());
      
      FramePose wayPointPose2 = new FramePose(leftSoleFrame);
      wayPointPose2.setX(0.45);
      wayPointPose2.setY(0.00);
      wayPointPose2.setZ(0.19);
      wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D locationWayPoint2 = new Point3D();
      wayPointPose2.getPose(locationWayPoint2, new Quaternion());
      
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);
      
      //footstepData.setSwingHeight(0.19);

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   @Override
   public void onBehaviorEntered()
   {
      abortBehavior.set(false);
      stepInPlace.set(true);
      sendPacket(new TextToSpeechPacket("Starting to step forward and backward with the right foot."));
   }

   @Override
   public void onBehaviorAborted()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      return abortBehavior.getBooleanValue();
   }
}
