package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
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
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
//import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.Hatch;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkThroughHatchBehavior extends StateMachineBehavior<WalkThroughHatchBehaviorState>
{
   private static final boolean useSafetyMarginForHatch = true;
   
   public enum WalkThroughHatchBehaviorState
   {
      STOPPED,
      SEARCHING_FOR_HATCH_FAR,
      INITIALIZE_TRAJECTORIES,
      INITIALIZE_ARM_LEFT,
      INITIALIZE_ARM_RIGHT,
      INITIALIZE_POSE,
      WALKING_TO_HATCH_FAR,
      SEARCHING_FOR_HATCH_NEAR,
      UPDATE_TRAJECTORIES,
      SETUP_ARMS_FOR_HATCH_WALK,
      SETUP_POSE_FOR_HATCH_WALK,
      WALKING_TO_HATCH_NEAR,
      ADJUST_CHEST,
      WAITING_FOR_CONFIRMATION,
      WALK_THROUGH_HATCH_FIRST_STEP,
      WALK_THROUGH_HATCH_TRANSITION,
      WALK_THROUGH_HATCH_SECOND_STEP,
      REALIGN_ROBOT,
      WALKING_FROM_HATCH_NEAR,
      STRAIGHTEN_UP,
      WALKING_FROM_HATCH_FAR,
      FAILED,
      DONE
   }
   
   private Point3D targetLocationHatchBeforeFar = new Point3D(-0.61, -0.08 + 0.08, 0.0); // -0.41 before changing for higher walk
   private Point3D targetLocationHatchBeforeNear = new Point3D(-0.20, -0.08 + 0.08, 0.0); // -0.19
   private Point3D targetLocationHatchAfterNear = new Point3D(0.60, -0.08 + 0.03 + 0.08, 0.0);
   private Point3D targetLocationHatchAfterFar = new Point3D(0.85, -0.08 + 0.03 + 0.08, 0.0);
   
   private final HumanoidReferenceFrames referenceFrames;
   
   private final double defaultLowerPelvisHeight = 0.788;
   private final double defaultUpperPelvisHeight = 0.850;
   
   private final PoseReferenceFrame hatchFrame = new PoseReferenceFrame("HatchFrame", ReferenceFrame.getWorldFrame());
   private final YoGraphicReferenceFrame hatchFrameViz;
   
//   private final int numberOfHatches = HatchEnvironment.getNumberOfHatches();
   private Hatch hatch;
   
   private final double minSwingWayPointHeight = 0.08;
   
   private final Point3D defaultLeftFootSwingWayPointBeforeHatch = new Point3D(0.03, 0.00, 0.06); // 0.07 z was 0.10
   private final Point3D defaultLeftFootSwingWayPointAfterHatch = new Point3D(-0.03, 0.00, 0.02); // z was 0.01 (0.09)
   private final Point3D defaultRightFootSwingWayPointBeforeHatch = new Point3D(0.03, 0.00, 0.04); // z was 0.04 (0.10)
   private final Point3D defaultRightFootSwingWayPointAfterHatch = new Point3D(-0.03, 0.00, 0.02); // z was 0.01 (0.09)
   
   private Vector3D defaultLeftFootSwingWaypointBeforeHatchOrientation = new Vector3D(0.0, Math.toRadians(30.0), 0.0);
   private Vector3D defaultLeftFootSwingWaypointAfterHatchOrientation = new Vector3D(0.0, Math.toRadians(0.0), 0.0);
   private Vector3D defaultRightFootSwingWaypointBeforeHatchOrientation = new Vector3D(0.0, Math.toRadians(0.0), 0.0);
   private Vector3D defaultRightFootSwingWaypointAfterHatchOrientation = new Vector3D(0.0, Math.toRadians(0.0), 0.0);
   
   private Point3D leftFootSwingWaypointBeforeHatchPosition = new Point3D();
   private Point3D leftFootSwingWaypointAfterHatchPosition = new Point3D();
   private Point3D rightFootSwingWaypointBeforeHatchPosition = new Point3D();
   private Point3D rightFootSwingWayPointAfterHatchPosition = new Point3D();
   
   private Point3D rightFootSwingGoalPosition = new Point3D();
   private Point3D rightFootBeforeHatchPositionOffset = new Point3D();
   private Point3D rightFootAfterHatchPositionOffset = new Point3D();
   
   private Point3D leftFootSwingGoalPosition = new Point3D();
   private Point3D leftFootBeforeHatchPositionOffset = new Point3D();
   private Point3D leftFootAfterHatchPositionOffset = new Point3D();
   
   private double[] chestYawPitchRollInitializeDesired = new double[3];
   private double[] chestYawPitchRollSetupDesired = new double[3];
   private double[] chestYawPitchRollAdjustDesired = new double[3];
   private double[] chestYawPitchRollFirstHatchStepDesired = new double[3];
   private double[] chestYawPitchRollTransitionDesired = new double[3];
   private double[] chestYawPitchRollSecondHatchStepDesired = new double[3];
   private double[] chestYawPitchRollResetDesired = new double[3];
   
   private double[] pelvisYawPitchRollInitializeDesired = new double[3];
   private double[] pelvisYawPitchRollSetupDesired = new double[3];
   private double[] pelvisYawPitchRollFirstHatchStepDesired = new double[3];
   private double[] pelvisYawPitchRollTransitionDesired = new double[3];
   private double[] pelvisYawPitchRollSecondHatchStepDesired = new double[3];
   private double[] pelvisYawPitchRollResetDesired = new double[3];
   
   private Point3D pelvisPositionInHatchFrameInitializeDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameSetupDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameFirstHatchStepDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameTransitionDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameSecondHatchStepDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameRealignDesired = new Point3D();
   private Point3D pelvisPositionInHatchFrameStraightenDesired = new Point3D();
   
   // Hatch feasibility boundaries
   private final double hatchWidthLowerBound = 0.86;
   private final double hatchUpperHeightLowerBound = 1.50;
   private final double hatchLowerHeightLowerBound = 0.05;
   private final double hatchLowerHeightUpperBound = 0.20;
   private final double hatchThicknessUpperBound = 0.15; // was 0.12
   
   // Trajectory timing constants
   private static final double defaulTime = 4.0;
   private final double initTime = defaulTime;
   private final double setupTime = defaulTime;
   private final double adjustTime = defaulTime;
   private final double firstStepTime = defaulTime;
   private final double transitionTime = defaulTime;
   private final double transitionTimePelvisHeight = 2.0;
   private final double secondStepChestTime = 2.0; //2.0;
   private final double secondStepTime = defaulTime;
   private final double realignTime = defaulTime;
   private final double straightenTime = defaulTime;
   
   private final double swingTimeHatch = 3.0;
   private final double transferTimeHatch = 0.6;
   
   private double defaultStepWidth;
   private double wideStepWidth = 0.20; //0.33
      
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);


//   private final WalkToLocationBehavior walkToLocationBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   
   private final SearchForHatchBehavior searchForHatchBehavior;

   RobotSide side = RobotSide.RIGHT;
   
   public WalkThroughHatchBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions, YoGraphicsListRegistry graphicsListRegistry)
   {
      super("walkThroughHatchBehavior", WalkThroughHatchBehaviorState.class, yoTime, communicationBridge);

      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      
      searchForHatchBehavior = new SearchForHatchBehavior(communicationBridge);
      
      swingTime.set(swingTimeHatch);
      transferTime.set(transferTimeHatch);

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
      
      defaultStepWidth = wholeBodyControllerParameters.getWalkingControllerParameters().getInPlaceWidth();
   }

   @Override
   public void doControl()
   {
      super.doControl();
   }

   private void setupStateMachine()
   {
      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatchFar = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH_FAR, searchForHatchBehavior)
      {         
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();

            setUpHatchFromEnvironment();
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializeTrajectories = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.INITIALIZE_TRAJECTORIES, new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         public void setBehaviorInput()
         {
            setRobotTrajectoriesBasedOnHatchDimensions();
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializeArmLeft = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.INITIALIZE_ARM_LEFT,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            
//            double[] leftArmPose = new double[] {-1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0};
//            double[] rightArmPose = new double[] {1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0};
            double[] leftArmPose = new double[] {-1.57, -0.20, 1.57, 1.57, -0.4, 1.25, 0.0};
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, initTime, leftArmPose);
            
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializeArmRight = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.INITIALIZE_ARM_RIGHT,
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);
            
//            double[] leftArmPose = new double[] {-1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0};
//            double[] rightArmPose = new double[] {1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0};
            double[] rightArmPose = new double[] {1.57, 0.80, 1.57, -1.57, 0.4, -1.25, 0.0};
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, initTime, rightArmPose);
            
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializePose = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.INITIALIZE_POSE,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, 
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollInitializeDesired);            
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(initTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(initTime, pelvisZUpFrame, pelvisYawPitchRollInitializeDesired);
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(initTime, hatchFrame, pelvisPositionInHatchFrameInitializeDesired);      

            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
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
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(defaultStepWidth);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatchNear = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH_NEAR, searchForHatchBehavior)
      {   
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();

            setUpHatchFromEnvironment();
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> updateTrajectories = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.UPDATE_TRAJECTORIES, new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         public void setBehaviorInput()
         {
            setRobotTrajectoriesBasedOnHatchDimensions();
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> setupArmsForHatchWalk = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_ARMS_FOR_HATCH_WALK, 
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);
            
            double[] leftArmPose = new double[] {-1.57, -0.20, 1.57, 1.57, -0.4, 1.25, 0.0}; //{-1.57, -0.51, 0.25, 2.0, 0.0, 0.0, 0.0}
            double[] rightArmPose = new double[] {1.57, 0.80, 1.57, -1.57, 0.4, -1.25, 0.0}; //{1.57, 0.51, 0.25, -2.0, 0.0, 0.0, 0.0}
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 1, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 1, leftArmPose);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> setupPoseForHatchWalk = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_POSE_FOR_HATCH_WALK,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {         
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollSetupDesired);
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(setupTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(setupTime, hatchFrame, pelvisYawPitchRollSetupDesired);
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(setupTime, hatchFrame, pelvisPositionInHatchFrameSetupDesired);
            
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
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(wideStepWidth);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> adjustChest = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.ADJUST_CHEST,
            atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollAdjustDesired);
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(adjustTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> waitForConfirmation = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WAITING_FOR_CONFIRMATION, searchForHatchBehavior)
      {   
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();

            setUpHatchFromEnvironment();
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchFirstStep = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP, atlasPrimitiveActions.footstepListBehavior, 
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollFirstHatchStepDesired);
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(firstStepTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(firstStepTime, hatchFrame, pelvisYawPitchRollFirstHatchStepDesired); // (was 1.5), 2.5
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(firstStepTime, hatchFrame, pelvisPositionInHatchFrameFirstHatchStepDesired); //3.0
            
            RobotSide robotSide = RobotSide.RIGHT;
            ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
            FramePose rightFootSwingGoalPose = new FramePose(soleFrame, rightFootSwingGoalPosition, new Quaternion());
            FramePose rightFootSwingWaypointBeforeHatchPose = new FramePose(soleFrame, rightFootSwingWaypointBeforeHatchPosition, new Quaternion(defaultRightFootSwingWaypointBeforeHatchOrientation));
            FramePose rightFootSwingWaypointAfterHatchPose = new FramePose(soleFrame, rightFootSwingWayPointAfterHatchPosition, new Quaternion(defaultRightFootSwingWaypointAfterHatchOrientation));
            FootstepDataListMessage footstepDataListMessage = getFootstepDataListMessage(robotSide, rightFootSwingGoalPose, rightFootSwingWaypointBeforeHatchPose, rightFootSwingWaypointAfterHatchPose);
                        
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footstepDataListMessage);
         }
      };
      
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchTransition = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {  
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollTransitionDesired);
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(transitionTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(transitionTime, hatchFrame, pelvisYawPitchRollTransitionDesired); //2.0
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(transitionTimePelvisHeight, hatchFrame, pelvisPositionInHatchFrameTransitionDesired); // 2.0
                        
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
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollSecondHatchStepDesired);
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(secondStepChestTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(secondStepTime, hatchFrame, pelvisYawPitchRollSecondHatchStepDesired); //2.0
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(secondStepTime, hatchFrame, pelvisPositionInHatchFrameSecondHatchStepDesired); //1.0            

            RobotSide robotSide = RobotSide.LEFT;
            ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
            FramePose leftFootSwingGoalPose = new FramePose(soleFrame, leftFootSwingGoalPosition, new Quaternion());
            FramePose leftFootSwingWaypointBeforeHatchPose = new FramePose(soleFrame, leftFootSwingWaypointBeforeHatchPosition, new Quaternion(defaultLeftFootSwingWaypointBeforeHatchOrientation));
            FramePose leftFootSwingWaypointAfterHatchPose = new FramePose(soleFrame, leftFootSwingWaypointAfterHatchPosition, new Quaternion(defaultLeftFootSwingWaypointAfterHatchOrientation));
            FootstepDataListMessage footstepDataListMessage = getFootstepDataListMessage(robotSide, leftFootSwingGoalPose, leftFootSwingWaypointBeforeHatchPose, leftFootSwingWaypointAfterHatchPose);

            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footstepDataListMessage);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> realign = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.REALIGN_ROBOT,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollResetDesired);
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(realignTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(realignTime, hatchFrame, pelvisYawPitchRollResetDesired); //1.0
            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(realignTime, hatchFrame, pelvisPositionInHatchFrameRealignDesired); //1.0 , was set to first step height           
            
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
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
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(defaultStepWidth);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> straighten = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.STRAIGHTEN_UP,
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(straightenTime, hatchFrame, pelvisPositionInHatchFrameStraightenDesired);  

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
            
            atlasPrimitiveActions.walkToLocationBehavior.setWalkingStepWidth(defaultStepWidth);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
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

      statemachine.addStateWithDoneTransition(searchForHatchFar, WalkThroughHatchBehaviorState.INITIALIZE_TRAJECTORIES);
      statemachine.addState(initializeTrajectories);
      initializeTrajectories.addStateTransition(WalkThroughHatchBehaviorState.INITIALIZE_ARM_LEFT, hatchPossibleCondition);
      initializeTrajectories.addStateTransition(WalkThroughHatchBehaviorState.FAILED, hatchImpossibleCondition);
      statemachine.addStateWithDoneTransition(initializeArmLeft, WalkThroughHatchBehaviorState.INITIALIZE_ARM_RIGHT);
      statemachine.addStateWithDoneTransition(initializeArmRight, WalkThroughHatchBehaviorState.INITIALIZE_POSE);
      statemachine.addStateWithDoneTransition(initializePose, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkToHatchFarAction, WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(searchForHatchNear, WalkThroughHatchBehaviorState.UPDATE_TRAJECTORIES);
      statemachine.addStateWithDoneTransition(updateTrajectories, WalkThroughHatchBehaviorState.SETUP_ARMS_FOR_HATCH_WALK);
      statemachine.addStateWithDoneTransition(setupArmsForHatchWalk, WalkThroughHatchBehaviorState.SETUP_POSE_FOR_HATCH_WALK);
      statemachine.addStateWithDoneTransition(setupPoseForHatchWalk, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkToHatchNearAction, WalkThroughHatchBehaviorState.ADJUST_CHEST);
      statemachine.addStateWithDoneTransition(adjustChest, WalkThroughHatchBehaviorState.WAITING_FOR_CONFIRMATION);
      statemachine.addStateWithDoneTransition(waitForConfirmation, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchFirstStep, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION);
      statemachine.addStateWithDoneTransition(walkThroughHatchTransition, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_SECOND_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchSecondStep, WalkThroughHatchBehaviorState.REALIGN_ROBOT);
      statemachine.addStateWithDoneTransition(realign, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkFromHatchNearAction, WalkThroughHatchBehaviorState.STRAIGHTEN_UP);
      statemachine.addStateWithDoneTransition(straighten, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkFromHatchFarAction, WalkThroughHatchBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(failedState, WalkThroughHatchBehaviorState.DONE);
      statemachine.addState(doneState);
      
      statemachine.setStartState(WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH_FAR);


   }
   
   
   public boolean currentHatchPossible()
   {
      return currentHatchDimensionsValid();
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
         PrintTools.debug("Opening " + hatch.getOpeningHeight() + "m x " + hatch.getWidth() + "m not possible");
      }
      if(!validHatchStepHeight())
      {
         PrintTools.debug("Step height " + hatch.getStepHeight() + "m not possible");
      }
      if(!validHatchThickness())
      {
         PrintTools.debug("Thickness " + hatch.getThickness() + "m not possible");
      }
      return false;
   }
   
   public boolean validHatchOpening()
   {
      return hatch.getWidth() >= hatchWidthLowerBound && hatch.getOpeningHeight() >= hatchUpperHeightLowerBound;
   }
   
   public boolean validHatchStepHeight()
   {
      return hatch.getStepHeight() >= hatchLowerHeightLowerBound && hatch.getStepHeight() <= hatchLowerHeightUpperBound;
   }
   
   public boolean validHatchThickness()
   {
      if(hatch.getThickness() <= hatchThicknessUpperBound)
      {
         if(hatch.getStepHeight() > hatchLowerHeightUpperBound && hatch.getThickness() > 0.05)
         {
            return false;
         }
         return true;
      }
      return false;
   }
   
   private void setUpHatchFromEnvironment()
   {
      hatch = new Hatch(searchForHatchBehavior.getLocation(), searchForHatchBehavior.getStepHeight(), searchForHatchBehavior.getOpeningHeight(),searchForHatchBehavior.getWidth(), searchForHatchBehavior.getThickness());            
      hatchFrame.setPoseAndUpdate(searchForHatchBehavior.getLocation());

      PrintTools.info("Found hatch at: " + searchForHatchBehavior.getLocation().getTranslationVector().toString());
      hatch.printHatchDimensions();
      
      if(useSafetyMarginForHatch)
      {
         hatch.applySafteyMargins();
      }
   }
   
   private void setRobotTrajectoriesBasedOnHatchDimensions()
   {        
      setChestTrajectoriesBasedOnHatchDimensions();
      setPelvisTrajectoriesBasedOnHatchDimensions();
      setFootTrajectoriesBasedOnHatchDimensions();
   }
   
   private void setChestTrajectoriesBasedOnHatchDimensions()
   {
      chestYawPitchRollInitializeDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollInitializeDesired[1] = Math.toRadians(10.0);
      chestYawPitchRollInitializeDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollSetupDesired[0] = Math.toRadians(20.0 - hatch.getStepHeight()/0.01); // 20.0 for 0.05 height (c = 25.0)
      chestYawPitchRollSetupDesired[1] = Math.toRadians(15.0);
      chestYawPitchRollSetupDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollAdjustDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollAdjustDesired[1] = Math.toRadians(15.0);
      chestYawPitchRollAdjustDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollFirstHatchStepDesired[0] = Math.toRadians(0.0); //-7.0
      chestYawPitchRollFirstHatchStepDesired[1] = Math.toRadians(27.0); //27.0
      chestYawPitchRollFirstHatchStepDesired[2] = Math.toRadians(0.0);
      
      chestYawPitchRollTransitionDesired[0] = Math.toRadians(0.0);
      chestYawPitchRollTransitionDesired[1] = Math.toRadians(20.0); //25.0
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
      pelvisYawPitchRollInitializeDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollInitializeDesired[1] = Math.toRadians(0.0);
      pelvisYawPitchRollInitializeDesired[2] = Math.toRadians(0.0);
      
      pelvisPositionInHatchFrameInitializeDesired.set(0.0, 0.0, defaultUpperPelvisHeight);
      
      pelvisYawPitchRollSetupDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollSetupDesired[1] = Math.toRadians(-hatch.getStepHeight()*100.0); //TEST: was -15.0 for 0.15 height
      pelvisYawPitchRollSetupDesired[2] = Math.toRadians(0.0);
      
      double pelvisMovementInitializationZ = -0.215 + 0.0475 * hatch.getStepHeight()/0.05 + (hatch.getOpeningHeight() - 1.55);
      if(pelvisMovementInitializationZ > 0.0)
         pelvisMovementInitializationZ = 0.0;
      pelvisPositionInHatchFrameSetupDesired.set(0.0, 0.0, defaultLowerPelvisHeight + pelvisMovementInitializationZ);
      
      pelvisYawPitchRollFirstHatchStepDesired[0] = Math.toRadians(10.0); //7.0
      pelvisYawPitchRollFirstHatchStepDesired[1] = Math.toRadians(-hatch.getStepHeight()*100.0); //5.0
      pelvisYawPitchRollFirstHatchStepDesired[2] = Math.toRadians(-7.0); // -7.0
      
      pelvisPositionInHatchFrameFirstHatchStepDesired.set(pelvisPositionInHatchFrameSetupDesired);
      pelvisPositionInHatchFrameFirstHatchStepDesired.add(0.0, 0.0, -0.02); //0.02 in z (0.05 in x)
      
      pelvisYawPitchRollTransitionDesired[0] = Math.toRadians(0.0); //0.0
      pelvisYawPitchRollTransitionDesired[1] = Math.toRadians(4.0 + 2.0 * hatch.getStepHeight()/0.05);
      pelvisYawPitchRollTransitionDesired[2] = Math.toRadians(5.0); // roll was 15.0 for 0.15 height

      pelvisPositionInHatchFrameTransitionDesired.set(pelvisPositionInHatchFrameFirstHatchStepDesired);
      pelvisPositionInHatchFrameTransitionDesired.add(0.0, 0.0, 0.05); // 0.04 height, 0.025 + 0.025*hatchLowerHeight/0.05 - 0.04 in x
   
      pelvisYawPitchRollSecondHatchStepDesired[0] = Math.toRadians(-8.0); // -5
      pelvisYawPitchRollSecondHatchStepDesired[1] = Math.toRadians(hatch.getStepHeight()*100.0);
      pelvisYawPitchRollSecondHatchStepDesired[2] = Math.toRadians(12.0); // 10
      
      pelvisPositionInHatchFrameSecondHatchStepDesired.set(pelvisPositionInHatchFrameTransitionDesired);
      pelvisPositionInHatchFrameSecondHatchStepDesired.add(0.0, 0.0, 0.0);    
      
      pelvisYawPitchRollResetDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[1] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[2] = Math.toRadians(0.0);
      
      pelvisPositionInHatchFrameRealignDesired.set(pelvisPositionInHatchFrameSecondHatchStepDesired); //pelvisPositionInHatchFrameFirstHatchStepDesired
      pelvisPositionInHatchFrameRealignDesired.add(0.0, 0.0, -0.01);
      
      pelvisPositionInHatchFrameStraightenDesired.set(0.0, 0.0, defaultUpperPelvisHeight);
   }
   
   private void setFootTrajectoriesBasedOnHatchDimensions()
   {
      setRelativeFootOffsetsFromHatch();
      setFootSwingGoalPointsBasedOnHatchDimensions();
      setFootSwingWayPointsBasedOnHatchDimensions();
   }
   
   private void setRelativeFootOffsetsFromHatch()
   {
      rightFootBeforeHatchPositionOffset.set(targetLocationHatchBeforeNear.getX(), 0.0, 0.0);
      rightFootAfterHatchPositionOffset.set(-rightFootBeforeHatchPositionOffset.getX() - 0.03, -0.08, 0.0); //x = -0.03 y=-0.12 ... -0.01 in x, 0.03 in y
      leftFootBeforeHatchPositionOffset.set(targetLocationHatchBeforeNear.getX(), 0.0, 0.0);
      leftFootAfterHatchPositionOffset.set(-leftFootBeforeHatchPositionOffset.getX() - 0.03, 0.03, 0.0); // +0.03 in x (-0.01 in x)
   }
   
   private void setFootSwingGoalPointsBasedOnHatchDimensions()
   {
      setRightFootSwingGoalPointBasedOnHatchDimensions();
      setLeftFootSwingGoalPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingGoalPointBasedOnHatchDimensions()
   {
      rightFootSwingGoalPosition.setToZero();
      rightFootSwingGoalPosition.sub(rightFootBeforeHatchPositionOffset);
      rightFootSwingGoalPosition.add(rightFootAfterHatchPositionOffset);
      rightFootSwingGoalPosition.add(new Point3D(hatch.getThickness(), 0.0, 0.0));
   }
   
   private void setLeftFootSwingGoalPointBasedOnHatchDimensions()
   {
      leftFootSwingGoalPosition.setToZero();
      leftFootSwingGoalPosition.sub(leftFootBeforeHatchPositionOffset);
      leftFootSwingGoalPosition.add(leftFootAfterHatchPositionOffset);
      leftFootSwingGoalPosition.add(new Point3D(hatch.getThickness(), 0.0, 0.0));
   }
   
   private void setFootSwingWayPointsBasedOnHatchDimensions()
   {
      setRightFootSwingWayPointBasedOnHatchDimensions();
      setLeftFootSwingWayPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingWayPointBasedOnHatchDimensions()
   {
      rightFootSwingWaypointBeforeHatchPosition.add(defaultRightFootSwingWayPointBeforeHatch, new Point3D(0, -0.03, hatch.getStepHeight())); // -0.03 in y
      
      rightFootSwingWayPointAfterHatchPosition.add(defaultRightFootSwingWayPointAfterHatch, new Point3D(0, -0.03, hatch.getStepHeight())); // -0.03 in y
      rightFootSwingWayPointAfterHatchPosition.add(rightFootSwingGoalPosition);
      
      if(rightFootSwingWayPointAfterHatchPosition.getZ() < minSwingWayPointHeight)
      {
         double deltaSwingHeight = minSwingWayPointHeight - rightFootSwingWayPointAfterHatchPosition.getZ();
         rightFootSwingWaypointBeforeHatchPosition.setZ(rightFootSwingWaypointBeforeHatchPosition.getZ() + deltaSwingHeight);
         rightFootSwingWayPointAfterHatchPosition.setZ(rightFootSwingWayPointAfterHatchPosition.getZ() + deltaSwingHeight);
      }
   }
   
   private void setLeftFootSwingWayPointBasedOnHatchDimensions()
   {
      leftFootSwingWaypointBeforeHatchPosition.add(defaultLeftFootSwingWayPointBeforeHatch, new Point3D(0, 0.00, hatch.getStepHeight())); // 0.03 ... was -0.02 in z
      
      leftFootSwingWaypointAfterHatchPosition.add(defaultLeftFootSwingWayPointAfterHatch, new Point3D(0, 0.05, hatch.getStepHeight())); // -->0.05 ... 0.03 in y
      leftFootSwingWaypointAfterHatchPosition.add(leftFootSwingGoalPosition);
      
      if(leftFootSwingWaypointAfterHatchPosition.getZ() < minSwingWayPointHeight)
      {
         double deltaSwingHeight = minSwingWayPointHeight - leftFootSwingWaypointAfterHatchPosition.getZ();
         leftFootSwingWaypointBeforeHatchPosition.setZ(leftFootSwingWaypointBeforeHatchPosition.getZ() + deltaSwingHeight);
         leftFootSwingWaypointAfterHatchPosition.setZ(leftFootSwingWaypointAfterHatchPosition.getZ() + deltaSwingHeight);
      }
   }
   
   public ChestTrajectoryMessage getChestOrientationTrajectoryMessage(double trajectoryTime, ReferenceFrame referenceFrame, double[] chestOrientation)
   {
      FrameOrientation chestOrientationFrame = new FrameOrientation(referenceFrame, chestOrientation);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      chestOrientationFrame.changeFrame(worldFrame);
      return new ChestTrajectoryMessage(trajectoryTime, chestOrientationFrame.getQuaternion(), worldFrame, worldFrame);
   }
   
   public PelvisOrientationTrajectoryMessage getPelvisOrientationTrajectoryMessage(double trajectoryTime, ReferenceFrame referenceFrame, double[] pelvisOrientation)
   {
      FrameOrientation pelvisOrientationFrame = new FrameOrientation(referenceFrame, pelvisOrientation);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      pelvisOrientationFrame.changeFrame(worldFrame);
      return new PelvisOrientationTrajectoryMessage(trajectoryTime, pelvisOrientationFrame.getQuaternion());
   }
   
   public PelvisHeightTrajectoryMessage getPelvisHeightTrajectoryMessage(double trajectoryTime, ReferenceFrame referenceFrame, Point3D pelvisPosition)
   {
      FramePose pelvisPoseDesired = new FramePose(referenceFrame, pelvisPosition, new Quaternion());
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      pelvisPoseDesired.changeFrame(worldFrame);
      return new PelvisHeightTrajectoryMessage(trajectoryTime, pelvisPoseDesired.getZ());
   }
   
   public FootstepDataListMessage getFootstepDataListMessage(RobotSide robotSide, FramePose footSwingGoalPose, FramePose footSwingWayPointBeforeHatchPose, FramePose footSwingWayPointAfterHatchPose)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      footSwingGoalPose.changeFrame(worldFrame);
      footSwingWayPointBeforeHatchPose.changeFrame(worldFrame);
      footSwingWayPointAfterHatchPose.changeFrame(worldFrame);
      
      SE3TrajectoryPointMessage[] swingTrajectoryPoints = new SE3TrajectoryPointMessage[2];
      Quaternion firstPointOrientation = new Quaternion();
      firstPointOrientation.setYawPitchRoll(new double[] {0.0, Math.toRadians(30.0), 0.0});
      swingTrajectoryPoints[0] = new SE3TrajectoryPointMessage(1.0 / 4.0 * swingTime.getDoubleValue(), footSwingWayPointBeforeHatchPose.getPosition(), footSwingWayPointBeforeHatchPose.getOrientation(), new Vector3D(), new Vector3D());
      swingTrajectoryPoints[1] = new SE3TrajectoryPointMessage(3.0 / 4.0 * swingTime.getDoubleValue(), footSwingWayPointAfterHatchPose.getPosition(), footSwingWayPointAfterHatchPose.getOrientation(), new Vector3D(), new Vector3D());
      
      FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, footSwingGoalPose.getPosition(), footSwingGoalPose.getOrientation());
      footstepData.setTrajectoryType(TrajectoryType.WAYPOINTS);
      footstepData.setSwingTrajectory(swingTrajectoryPoints);

      footsteps.add(footstepData);
      return footsteps;
   }
   
   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
   }
   
   @Override
   public void onBehaviorExited()
   {
      
   }
   

}
