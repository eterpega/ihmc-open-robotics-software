package us.ihmc.humanoidBehaviors.behaviors.debug;

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
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;

public class TestHatchWalkthroughBehavior extends AbstractBehavior
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyTransform hatchToWorld = new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset());
   private final ReferenceFrame hatchFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("HatchFrame", worldFrame, hatchToWorld);
   
   private final HumanoidReferenceFrames referenceFrames;
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable sleepTime = new DoubleYoVariable("BehaviorSleepTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);
   private final DoubleYoVariable stepLength = new DoubleYoVariable("BehaviorStepLength", registry);
   private final BooleanYoVariable stepInPlace = new BooleanYoVariable("StepInPlace", registry);
   private final BooleanYoVariable abortBehavior = new BooleanYoVariable("AbortBehavior", registry);

   private final YoStopwatch timer;
   
   double armTrajectoryTime; // was 4.0
   double leftArmGoalPosition[] = new double[] { -1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0 };
   double rightArmGoalPosition[] = new double[] { 1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0 };
   boolean madeFirstStepThroughHatch = false;
   boolean madeSecondStepThroughHatch = false;
   boolean robotConfigurationInitialized = false;
   boolean changedRobotConfiguration = false;
   double counterHelper = 0;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(10);

   private int version = 2;

   public TestHatchWalkthroughBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime)
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
      }
   }
   
   public void initializeRobotConfigurationCoordinated()
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
   
   public void makeFirstStepThroughHatchOpeningCoordinated()
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
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
      footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);


      sendPacket(footsteps);
      madeFirstStepThroughHatch = true;
   }
   
   public void changeRobotConfigurationCoordinated()
   {
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(25.0), Math.toRadians(0.0));
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
   
   public void makeSecondStepThroughHatchOpeningCoordinated()
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
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
      footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
      footsteps.add(footstepData);
      

      sendPacket(footsteps);
      madeSecondStepThroughHatch = true;
   }
   
   
   
   
   
   
   
   
   
   
   
   
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
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
      footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
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
      footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      
      
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
      footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
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
