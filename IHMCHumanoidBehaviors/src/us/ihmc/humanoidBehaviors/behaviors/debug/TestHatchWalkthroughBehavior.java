package us.ihmc.humanoidBehaviors.behaviors.debug;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.behaviorServices.BehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.YoStopwatch;

public class TestHatchWalkthroughBehavior extends AbstractBehavior
{
   private final HumanoidReferenceFrames referenceFrames;
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable sleepTime = new DoubleYoVariable("BehaviorSleepTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);
   private final DoubleYoVariable stepLength = new DoubleYoVariable("BehaviorStepLength", registry);
   private final BooleanYoVariable stepInPlace = new BooleanYoVariable("StepInPlace", registry);
   private final BooleanYoVariable abortBehavior = new BooleanYoVariable("AbortBehavior", registry);

   private final YoStopwatch timer;
   
   double armTrajectoryTime = 4.0;
   double leftArmGoalPosition[] = new double[] { -1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0 };
   double rightArmGoalPosition[] = new double[] { 1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0 };
   boolean madeFirstStepThroughHatch = false;
   boolean madeSecondStepThroughHatch = false;
   boolean robotConfigurationInitialized = false;
   double counterHelper = 0;
   private final ConcurrentListeningQueue<FootstepStatus> footstepStatusQueue = new ConcurrentListeningQueue<FootstepStatus>(10);


   public TestHatchWalkthroughBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime)
   {
      super(communicationBridge);
      this.referenceFrames = referenceFrames;

      swingTime.set(1.2);
      transferTime.set(0.6);
      sleepTime.set(12.0);
      stepLength.set(0.3);

      timer = new YoStopwatch(yoTime);
   }
   
   @Override
   public void doControl()
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
      else if(!madeSecondStepThroughHatch && (timer.totalElapsed() > (counterHelper + 1.5 * armTrajectoryTime)))
      {
         makeSecondStepThroughHatchOpening();
      }
   }
   
   public void initializeRobotConfiguration()
   {
      AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0));
      Quaternion chestOrientation = new Quaternion(chestOrientationAA);
      ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(armTrajectoryTime, chestOrientation);
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
      AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); //was 10.0, (might have) worked
      Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
      PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(1.5*armTrajectoryTime, pelvisOrientation);
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
   
   public void makeSecondStepThroughHatchOpening()
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);

      ReferenceFrame leftSoleFrame = referenceFrames.getSoleFrame(RobotSide.LEFT);
      FramePose stepPose = new FramePose(leftSoleFrame);
      stepPose.setX(0.58);
      stepPose.setY(-0.08);

      stepPose.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);

      FootstepDataMessage footstepData = new FootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      footstepData.setSwingHeight(0.17);

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
