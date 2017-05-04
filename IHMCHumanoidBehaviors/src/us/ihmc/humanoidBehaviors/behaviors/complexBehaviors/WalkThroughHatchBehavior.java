package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughHatchBehavior.WalkThroughHatchBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
//import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkThroughHatchBehavior extends StateMachineBehavior<WalkThroughHatchBehaviorState>
{
   private static final boolean fakeHatch = false;
   
   public enum WalkThroughHatchBehaviorState
   {
      STOPPED,
      SEARCHING_FOR_HATCH,
      SETUP_ROBOT_ARMS,
      WALKING_TO_HATCH_FAR,
      SETUP_ROBOT,
      WALKING_TO_HATCH_NEAR,
      ADJUST_CHEST,
      WALK_THROUGH_HATCH_FIRST_STEP,
      WALK_THROUGH_HATCH_TRANSITION,
      WALK_THROUGH_HATCH_SECOND_STEP,
      WALKING_FROM_HATCH_NEAR,
      RESET_ROBOT,
      WALKING_FROM_HATCH_FAR,
      FAILED,
      DONE
   }
   
   private final boolean useSafetyMarginForHatch = true;
   
   private Vector3D targetLocationHatchBeforeFar = new Vector3D(-0.61, -0.08 + 0.02, 0.0);
   private Vector3D targetLocationHatchBeforeNear = new Vector3D(-0.21, -0.08 + 0.02, 0.0);
   private Vector3D targetLocationHatchAfterNear = new Vector3D(0.60, -0.08 + 0.03 + 0.02, 0.0);
   private Vector3D targetLocationHatchAfterFar = new Vector3D(0.85, -0.08 + 0.03 + 0.02, 0.0);
   
   private final HumanoidReferenceFrames referenceFrames;
   
   private final double defaultPelvisHeight = 0.788;
   
   private final PoseReferenceFrame hatchFrame = new PoseReferenceFrame("HatchFrame", ReferenceFrame.getWorldFrame());
   private final YoGraphicReferenceFrame hatchFrameViz;
   
   private final int numberOfHatches = HatchEnvironment.getNumberOfHatches();
   private int currentHatch = 1;
   private double hatchWidth;
   private double hatchThickness;
   private double hatchLowerHeight;
   private double hatchUpperHeight;
   
   private final Point3D defaultFootSwingWayPoint1 = new Point3D(0.03, 0.00, 0.10); // z was 0.10
   private final Point3D defaultFootSwingWayPoint2 = new Point3D(-0.03, 0.00, 0.09); // z was 0.08
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
   
   private double[] chestYawPitchRollInitializeDesired = new double[3];
   private double[] chestYawPitchRollSetupDesired = new double[3];
   private double[] chestYawPitchRollAdjustDesired = new double[3];
   private double[] chestYawPitchRollFirstHatchStepDesired = new double[3];
   private double[] chestYawPitchRollTransitionDesired = new double[3];
   private double[] chestYawPitchRollSecondHatchStepDesired = new double[3];
   private double[] chestYawPitchRollResetDesired = new double[3];
   
   private double[] pelvisYawPitchRollSetupDesired = new double[3];
   private double[] pelvisYawPitchRollFirstHatchStepDesired = new double[3];
   private double[] pelvisYawPitchRollTransitionDesired = new double[3];
   private double[] pelvisYawPitchRollSecondHatchStepDesired = new double[3];
   private double[] pelvisYawPitchRollResetDesired = new double[3];
   
   private Vector3D pelvisPositionInHatchFrameSetupDesired = new Vector3D();
   private Vector3D pelvisPositionInHatchFrameFirstHatchStepDesired = new Vector3D();
   private Vector3D pelvisPositionInHatchFrameTransitionDesired = new Vector3D();
   private Vector3D pelvisPositionInHatchFrameSecondHatchStepDesired = new Vector3D();
   private Vector3D pelvisPositionInHatchFrameResetDesired = new Vector3D();
   
   private final double hatchWidthLowerBound = 0.86;
   private final double hatchUpperHeightLowerBound = 1.55;
   private final double hatchLowerHeightLowerBound = 0.05;
   private final double hatchLowerHeightUpperBound = 0.20;
   private final double hatchThicknessUpperBound = 0.12;
   
      
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);


//   private final WalkToLocationBehavior walkToLocationBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   RobotSide side = RobotSide.RIGHT;
   
   public WalkThroughHatchBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions, YoGraphicsListRegistry graphicsListRegistry)
   {
      super("walkThroughHatchBehavior", WalkThroughHatchBehaviorState.class, yoTime, communicationBridge);

      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      
      swingTime.set(1.2);
      transferTime.set(0.6);

      setupStateMachine();
      
      if (graphicsListRegistry != null)
      {
         hatchFrameViz = new YoGraphicReferenceFrame(hatchFrame, registry, 0.5);
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), hatchFrameViz);
      }
      else
      {
         hatchFrameViz = null;
      }
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for door and updating its location here
      super.doControl();
   }

   private void setupStateMachine()
   {
      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatch = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH, new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         public void setBehaviorInput()
         {
            if(currentHatchExists())
            {
               setRobotTrajectoriesBasedOnHatchDimensions(currentHatch);
            }
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> setupRobotArms = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_ROBOT_ARMS,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Hands
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);
            
            // Arms
            double[] leftArmPose = new double[] {-1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0};
            double[] rightArmPose = new double[] {1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0};
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftArmPose);
            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollInitializeDesired[0], chestYawPitchRollInitializeDesired[1], chestYawPitchRollInitializeDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());

            // Send commands
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkToHatchFarAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_TO_HATCH_FAR, atlasPrimitiveActions.walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePose2d targetPose = new FramePose2d(hatchFrame);
            targetPose.setX(targetLocationHatchBeforeFar.getX());
            targetPose.setY(targetLocationHatchBeforeFar.getY());
            targetPose.setYaw(0.0);
            targetPose.changeFrame(ReferenceFrame.getWorldFrame());
            
//            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(0.33);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> setup = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_ROBOT,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, 
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // Hands
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);
            
            // Arms
            double[] leftArmPose = new double[] {-1.57, -0.51, 0.25, 2.0, 0.0, 0.0, 0.0};
            double[] rightArmPose = new double[] {1.57, 0.51, 0.25, -2.0, 0.0, 0.0, 0.0};
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 1, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 1, leftArmPose);
            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollSetupDesired[0], chestYawPitchRollSetupDesired[1], chestYawPitchRollSetupDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisYawPitchRollSetupDesired[0], pelvisYawPitchRollSetupDesired[1], pelvisYawPitchRollSetupDesired[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(1.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisPositionInHatchFrameSetupDesired);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            FramePose pelvisPoseDesired = new FramePose(hatchFrame, pelvisPositionInHatchFrameSetupDesired, new Quaternion());
            pelvisPoseDesired.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.0, pelvisPoseDesired.getZ());            
            
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> walkToHatchNearAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_TO_HATCH_NEAR, atlasPrimitiveActions.walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePose2d targetPose = new FramePose2d(hatchFrame);
            targetPose.setX(targetLocationHatchBeforeNear.getX());
            targetPose.setY(targetLocationHatchBeforeNear.getY());
            targetPose.setYaw(0.0);
            targetPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(0.33);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> adjustChest = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.ADJUST_CHEST,
            atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(15.0), 0.0); // WAS 17.0, BKY complained during walking
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());

            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchFirstStep = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP, atlasPrimitiveActions.footstepListBehavior, 
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollFirstHatchStepDesired[0], chestYawPitchRollFirstHatchStepDesired[1], chestYawPitchRollFirstHatchStepDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisYawPitchRollFirstHatchStepDesired[0], pelvisYawPitchRollFirstHatchStepDesired[1], pelvisYawPitchRollFirstHatchStepDesired[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.5, pelvisOrientationWorld); // was 1.5
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisPositionInHatchFrameFirstHatchStepDesired);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            FramePose pelvisPoseDesired = new FramePose(hatchFrame, pelvisPositionInHatchFrameFirstHatchStepDesired, new Quaternion());
            pelvisPoseDesired.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(3.0, pelvisPoseDesired.getZ());

            // Feet
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
//            footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

            wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint1 = new Point3D();
            wayPointPose1.getPose(locationWayPoint1, new Quaternion());
            wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint2 = new Point3D();
            wayPointPose2.getPose(locationWayPoint2, new Quaternion());
            
            footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
            footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
            footsteps.add(footstepData);
                        
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footsteps);
         }
      };
      
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchTransition = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollTransitionDesired[0], chestYawPitchRollTransitionDesired[1], chestYawPitchRollTransitionDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3.0, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisYawPitchRollTransitionDesired[0], pelvisYawPitchRollTransitionDesired[1], pelvisYawPitchRollTransitionDesired[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            FramePose pelvisPoseDesired = new FramePose(hatchFrame, pelvisPositionInHatchFrameTransitionDesired, new Quaternion());
            pelvisPoseDesired.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(2.0, pelvisPoseDesired.getZ());
            
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
         }
      };
      
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchSecondStep = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_SECOND_STEP, atlasPrimitiveActions.footstepListBehavior, 
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollSecondHatchStepDesired[0], chestYawPitchRollSecondHatchStepDesired[1], chestYawPitchRollSecondHatchStepDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisYawPitchRollSecondHatchStepDesired[0], pelvisYawPitchRollSecondHatchStepDesired[1], pelvisYawPitchRollSecondHatchStepDesired[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            FramePose pelvisPoseDesired = new FramePose(hatchFrame, pelvisPositionInHatchFrameFirstHatchStepDesired, new Quaternion());
            pelvisPoseDesired.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.0, pelvisPoseDesired.getZ());
            
            // Feet
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
//            footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
            
            wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint1 = new Point3D();
            wayPointPose1.getPose(locationWayPoint1, new Quaternion());
            wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint2 = new Point3D();
            wayPointPose2.getPose(locationWayPoint2, new Quaternion());
            
            footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
            footstepData.setCustomPositionWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
            footsteps.add(footstepData);
            
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footsteps);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkFromHatchNearAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_NEAR, atlasPrimitiveActions.walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePose2d targetPose = new FramePose2d(hatchFrame);
            targetPose.setX(targetLocationHatchAfterNear.getX());
            targetPose.setY(targetLocationHatchAfterNear.getY());
            targetPose.setYaw(0.0);
            targetPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(0.33);
            atlasPrimitiveActions.walkToLocationBehavior.setFootstepLength(0.20);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> reset = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.RESET_ROBOT,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollResetDesired[0], chestYawPitchRollResetDesired[1], chestYawPitchRollResetDesired[2]);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3.0, chestOrientationLocal, pelvisZUpFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisYawPitchRollResetDesired[0], pelvisYawPitchRollResetDesired[1], pelvisYawPitchRollResetDesired[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(1.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisPositionInHatchFrameResetDesired);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            FramePose pelvisPoseDesired = new FramePose(hatchFrame, pelvisPositionInHatchFrameResetDesired, new Quaternion());
            pelvisPoseDesired.changeFrame(ReferenceFrame.getWorldFrame());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.0, pelvisPoseDesired.getZ());            
            
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
         }
      };
      
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkFromHatchFarAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_FAR, atlasPrimitiveActions.walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePose2d targetPose = new FramePose2d(hatchFrame);
            targetPose.setX(targetLocationHatchAfterFar.getX());
            targetPose.setY(targetLocationHatchAfterFar.getY());
            targetPose.setYaw(0.0);
            targetPose.changeFrame(ReferenceFrame.getWorldFrame());
            
//            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(0.33);
            atlasPrimitiveActions.walkToLocationBehavior.setFootstepLength(0.20);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
            
            ++currentHatch;
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> failedState = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.FAILED,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking Through Hatch Failed");
            sendPacket(p1);
            
            PrintTools.info(this, "No more possible hatches");
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> doneState = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.DONE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Finished Walking Through Hatch");
            sendPacket(p1);
            
            PrintTools.debug(this, "Done");
         }
      };
      
      StateTransitionCondition hatchPossibleCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return currentHatchPossible();
         }
      };
      
      StateTransitionCondition hatchImpossibleCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return !currentHatchPossible();
         }
      };

//      statemachine.addStateWithDoneTransition(searchForHatch, WalkThroughHatchBehaviorState.SETUP_ROBOT_ARMS);
      statemachine.addState(searchForHatch);
      searchForHatch.addStateTransition(WalkThroughHatchBehaviorState.SETUP_ROBOT_ARMS, hatchPossibleCondition);
      searchForHatch.addStateTransition(WalkThroughHatchBehaviorState.FAILED, hatchImpossibleCondition);
      statemachine.addStateWithDoneTransition(setupRobotArms, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkToHatchFarAction, WalkThroughHatchBehaviorState.SETUP_ROBOT);
      statemachine.addStateWithDoneTransition(setup, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkToHatchNearAction, WalkThroughHatchBehaviorState.ADJUST_CHEST);
      statemachine.addStateWithDoneTransition(adjustChest, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchFirstStep, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION);
      statemachine.addStateWithDoneTransition(walkThroughHatchTransition, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_SECOND_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchSecondStep, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkFromHatchNearAction, WalkThroughHatchBehaviorState.RESET_ROBOT);
      statemachine.addStateWithDoneTransition(reset, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkFromHatchFarAction, WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH);
      statemachine.addStateWithDoneTransition(failedState, WalkThroughHatchBehaviorState.DONE);
      statemachine.addState(doneState);
      
      statemachine.setStartState(WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH);

   }
   
   
   public boolean currentHatchPossible()
   {
      if(currentHatchExists())
      {
         return currentHatchDimensionsValid();
      }
      PrintTools.debug("Hatch not possible");
      return false;
   }
   
   public boolean currentHatchExists()
   {
      if(currentHatch > numberOfHatches)
      {
         PrintTools.debug("Hatch does not exist");
         return false;
      }
      return true;
   }
   
   public boolean currentHatchDimensionsValid()
   {
      if(validHatchOpening() && validHatchStepHeight() && validHatchThickness())
      {
         return true;
      }
      PrintTools.debug("Hatch invalid dimensions");
      if(!validHatchOpening())
      {
         PrintTools.debug("Opening not possible");
      }
      if(!validHatchStepHeight())
      {
         PrintTools.debug("Step not possible");
      }
      if(!validHatchThickness())
      {
         PrintTools.debug("Thickness not possible");
      }
      return false;
   }
   
   public boolean validHatchOpening()
   {
      return hatchWidth >= hatchWidthLowerBound && hatchUpperHeight >= hatchUpperHeightLowerBound;
   }
   
   public boolean validHatchStepHeight()
   {
      return hatchLowerHeight >= hatchLowerHeightLowerBound && hatchLowerHeight <= hatchLowerHeightUpperBound;
   }
   
   public boolean validHatchThickness()
   {
      if(hatchThickness <= hatchThicknessUpperBound)
      {
         if(hatchLowerHeight > 0.15 && hatchThickness > 0.05)
         {
            return false;
         }
         return true;
      }
      return false;
   }
   
   private void setRobotTrajectoriesBasedOnHatchDimensions(int hatch)
   {  
      int relHatch = hatch - 1;
      
      if (fakeHatch)
      {
         FramePose hatchPose = new FramePose(referenceFrames.getMidFeetZUpFrame());
         hatchPose.setX(1.0);
         hatchPose.changeFrame(ReferenceFrame.getWorldFrame());
         hatchFrame.setPoseAndUpdate(hatchPose);
      }
      else
      {
         Point3D hatchFrameOffset = HatchEnvironment.getHatchFrameOffset(relHatch);
         FramePose hatchPose = new FramePose(ReferenceFrame.getWorldFrame());
         hatchPose.setPosition(hatchFrameOffset);
         hatchFrame.setPoseAndUpdate(hatchPose);
      }
      
      hatchFrameViz.update();
      
      FramePoint hatchOrigin = new FramePoint(hatchFrame);
      hatchOrigin.changeFrame(ReferenceFrame.getWorldFrame());
      PrintTools.info("Found hatch at: " + hatchOrigin.toString());
      
      hatchWidth = HatchEnvironment.getHatchWidth(relHatch);
      hatchThickness = HatchEnvironment.getHatchThickness(relHatch);
      hatchLowerHeight = HatchEnvironment.getHatchLowerHeight(relHatch);
      hatchUpperHeight = HatchEnvironment.getHatchUpperHeight(relHatch);
      
      if(useSafetyMarginForHatch)
      {
         hatchUpperHeight = hatchUpperHeight - 0.05;
         hatchThickness = hatchThickness + 0.01;
      }
      
      rightBeforeHatchOffset.set(0.21, 0.0, 0.0);
      rightAfterHatchOffset.set(rightBeforeHatchOffset.getX() - 0.01, 0.03, 0.0); // -0.01 in x
      leftBeforeHatchOffset.set(0.21, 0.0, 0.0);
      leftAfterHatchOffset.set(leftBeforeHatchOffset.getX() + 0.03, 0.03, 0.0); // -0.01 in x
      
      setChestTrajectoriesBasedOnHatchDimensions();
      setPelvisTrajectoriesBasedOnHatchDimensions();
      setFootSwingGoalPointsBasedOnHatchDimensions();
      setFootSwingWayPointsBasedOnHatchDimensions();
   }
   
   private void setChestTrajectoriesBasedOnHatchDimensions()
   {
      chestYawPitchRollInitializeDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollInitializeDesired[1] = Math.toRadians(10.0);
      chestYawPitchRollInitializeDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollSetupDesired[0] = Math.toRadians(20.0 - hatchLowerHeight/0.01); // 20.0 for 0.05 height (c = 25.0)
      chestYawPitchRollSetupDesired[1] = Math.toRadians(15.0);
      chestYawPitchRollSetupDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollAdjustDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollAdjustDesired[1] = Math.toRadians(15.0);
      chestYawPitchRollAdjustDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollFirstHatchStepDesired[0] = Math.toRadians(-7.0);
      chestYawPitchRollFirstHatchStepDesired[1] = Math.toRadians(27.0);
      chestYawPitchRollFirstHatchStepDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollTransitionDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollTransitionDesired[1] = Math.toRadians(25.0);
      chestYawPitchRollTransitionDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollSecondHatchStepDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollSecondHatchStepDesired[1] = Math.toRadians(7.0);
      chestYawPitchRollSecondHatchStepDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollResetDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollResetDesired[1] = Math.toRadians(7.0);
      chestYawPitchRollResetDesired[2] = Math.toRadians(0.0);
   }
   
   private void setPelvisTrajectoriesBasedOnHatchDimensions()
   {
      pelvisYawPitchRollSetupDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollSetupDesired[1] = Math.toRadians(-hatchLowerHeight*100.0); //TEST: was -15.0 for 0.15 height
      pelvisYawPitchRollSetupDesired[2] = Math.toRadians(0.0);
      
      double pelvisMovementInitializationZ = -0.215 + 0.0475 * hatchLowerHeight/0.05 + (hatchUpperHeight - 1.55);
      if(pelvisMovementInitializationZ > 0.0)
         pelvisMovementInitializationZ = 0.0;
      pelvisPositionInHatchFrameSetupDesired.set(0.0, 0.0, defaultPelvisHeight + pelvisMovementInitializationZ);
      
      pelvisYawPitchRollFirstHatchStepDesired[0] = Math.toRadians(7.0);
      pelvisYawPitchRollFirstHatchStepDesired[1] = Math.toRadians(5.0);
      pelvisYawPitchRollFirstHatchStepDesired[2] = Math.toRadians(-7.0); // -7.0
      
      pelvisPositionInHatchFrameFirstHatchStepDesired.set(pelvisPositionInHatchFrameSetupDesired);
      pelvisPositionInHatchFrameFirstHatchStepDesired.add(0.0, 0.0, 0.02); //0.05 in x
      
      pelvisYawPitchRollTransitionDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollTransitionDesired[1] = Math.toRadians(4.0 + 2.0 * hatchLowerHeight/0.05);
      pelvisYawPitchRollTransitionDesired[2] = Math.toRadians(0.0); // roll was 15.0 for 0.15 height

      pelvisPositionInHatchFrameTransitionDesired.set(pelvisPositionInHatchFrameFirstHatchStepDesired);
      pelvisPositionInHatchFrameTransitionDesired.add(0.0, 0.0, 0.04); // 0.03 height, 0.025 + 0.025*hatchLowerHeight/0.05 - 0.04 in x
   
      pelvisYawPitchRollSecondHatchStepDesired[0] = Math.toRadians(-8.0); // -5
      pelvisYawPitchRollSecondHatchStepDesired[1] = Math.toRadians(hatchLowerHeight*100.0);
      pelvisYawPitchRollSecondHatchStepDesired[2] = Math.toRadians(12.0); // 10
      
      pelvisPositionInHatchFrameSecondHatchStepDesired.set(pelvisPositionInHatchFrameTransitionDesired);
      pelvisPositionInHatchFrameSecondHatchStepDesired.add(0.0, 0.0, 0.0);    
      
      pelvisYawPitchRollResetDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[1] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[2] = Math.toRadians(0.0);
      
      pelvisPositionInHatchFrameResetDesired.set(0.0, 0.0, defaultPelvisHeight);
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
      leftFootSwingWayPoint1.add(defaultFootSwingWayPoint1, new Point3D(0, 0.03, hatchLowerHeight - 0.02)); // was -0.02 in z
      leftFootSwingWayPoint2.add(defaultFootSwingWayPoint2, new Point3D(0, 0.03, hatchLowerHeight));
   }
   
   @Override
   public void onBehaviorExited()
   {
      
   }
   

}
