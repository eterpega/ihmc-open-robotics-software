package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
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
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HatchLocationPacket;
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
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.HatchEnvironment.Hatch;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkThroughHatchBehavior extends StateMachineBehavior<WalkThroughHatchBehaviorState>
{
   private static final boolean useSafetyMarginForHatch = true;
   
   public enum WalkThroughHatchBehaviorState
   {
      STOPPED,
      SEARCHING_FOR_HATCH,
      INITIALIZE_TRAJECTORIES,
      INITIALIZE_CONFIGURATION,
      WALKING_TO_HATCH_FAR,
      SETUP_FOR_HATCH_WALK,
      WALKING_TO_HATCH_NEAR,
      ADJUST_CHEST,
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
   
   private Point3D targetLocationHatchBeforeFar = new Point3D(-0.41, -0.08 + 0.02, 0.0); // -0.61 before changing for higher walk
   private Point3D targetLocationHatchBeforeNear = new Point3D(-0.19, -0.08 + 0.02, 0.0); // -0.21
   private Point3D targetLocationHatchAfterNear = new Point3D(0.60, -0.08 + 0.03 + 0.02, 0.0);
   private Point3D targetLocationHatchAfterFar = new Point3D(0.85, -0.08 + 0.03 + 0.02, 0.0);
   
   private final HumanoidReferenceFrames referenceFrames;
   
   private final double defaultLowerPelvisHeight = 0.788;
   private final double defaultUpperPelvisHeight = 0.850;
   
   private final PoseReferenceFrame hatchFrame = new PoseReferenceFrame("HatchFrame", ReferenceFrame.getWorldFrame());
   private final YoGraphicReferenceFrame hatchFrameViz;
   
   private final int numberOfHatches = HatchEnvironment.getNumberOfHatches();
   private int currentHatch = 1;
   private double hatchWidth;
   private double hatchThickness;
   private double hatchLowerHeight;
   private double hatchUpperHeight;
   
   private final double minSwingWayPointHeight = 0.08;
   
   private final Point3D defaultLeftFootSwingWayPoint1 = new Point3D(0.03, 0.00, 0.10); // z was 0.10
   private final Point3D defaultLeftFootSwingWayPoint2 = new Point3D(-0.03, 0.00, 0.00); // z was 0.09
   private final Point3D defaultRightFootSwingWayPoint1 = new Point3D(0.03, 0.00, 0.00); // z was 0.10
   private final Point3D defaultRightFootSwingWayPoint2 = new Point3D(-0.03, 0.00, 0.00); // z was 0.09
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
   
   // Hatch feasability boundaries
   private final double hatchWidthLowerBound = 0.86;
   private final double hatchUpperHeightLowerBound = 1.55;
   private final double hatchLowerHeightLowerBound = 0.05;
   private final double hatchLowerHeightUpperBound = 0.20;
   private final double hatchThicknessUpperBound = 0.12;
   
   // Trajectory timing constants
   private static final double defaulTime = 4.0;
   private final double initTime = defaulTime;
   private final double setupTime = defaulTime;
   private final double adjustTime = defaulTime;
   private final double firstStepTime = defaulTime;
   private final double transitionTime = defaulTime;
   private final double secondStepChestTime = 2.0;
   private final double secondStepTime = defaulTime;
   private final double realignTime = defaulTime;
   private final double straightenTime = defaulTime;
   
   private final double swingTimeHatch = 3.0;
   private final double transferTimeHatch = 0.6;
   
   private double defaultStepWidth;
   private double wideStepWidth = 0.33;
      
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
      //should constantly be searching for door and updating its location here
      super.doControl();
   }

   private void setupStateMachine()
   {
      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatch = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH, searchForHatchBehavior)
      {         
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();

//            HatchLocationPacket hatchLocationPacket = new HatchLocationPacket(searchForHatchBehavior.getLocation());
//            communicationBridge.sendPacketToUI(hatchLocationPacket);
            hatchFrame.setPoseAndUpdate(searchForHatchBehavior.getLocation());
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializeTrajectories = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.INITIALIZE_TRAJECTORIES, new SimpleDoNothingBehavior(communicationBridge))
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
      
      BehaviorAction<WalkThroughHatchBehaviorState> initializeConfiguration = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.INITIALIZE_CONFIGURATION,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, 
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
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
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, initTime, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, initTime, leftArmPose);
            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollInitializeDesired);            
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(initTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(initTime, pelvisZUpFrame, pelvisYawPitchRollInitializeDesired);
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(initTime, hatchFrame, pelvisPositionInHatchFrameInitializeDesired);      
            
            // Send commands
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
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
            
            PrintTools.debug("Walking to hatch far");
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> setupForHatchWalk = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_FOR_HATCH_WALK,
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
            
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollSetupDesired);
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(setupTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(setupTime, hatchFrame, pelvisYawPitchRollSetupDesired);
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(setupTime, hatchFrame, pelvisPositionInHatchFrameSetupDesired);
            
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
            // Chest            
//            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
//            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollAdjustDesired);
//            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(adjustTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            FrameOrientation chestOrientationFrame = new FrameOrientation(hatchFrame, chestYawPitchRollAdjustDesired);
            ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
            chestOrientationFrame.changeFrame(worldFrame);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(adjustTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
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
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollFirstHatchStepDesired);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(firstStepTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame()); //2.5
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(firstStepTime, pelvisZUpFrame, pelvisYawPitchRollFirstHatchStepDesired); // (was 1.5), 2.5
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(firstStepTime, hatchFrame, pelvisPositionInHatchFrameFirstHatchStepDesired); //3.0
            FootstepDataListMessage footstepDataListMessage = getFootstepDataListMessage(RobotSide.RIGHT, rightFootSwingGoalPoint, rightFootSwingWayPoint1, rightFootSwingWayPoint2);
                        
            // Send commands
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
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollTransitionDesired);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(transitionTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame()); //3.0
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(transitionTime, pelvisZUpFrame, pelvisYawPitchRollTransitionDesired); //2.0
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(transitionTime, hatchFrame, pelvisPositionInHatchFrameTransitionDesired); // 2.0
                        
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
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollSecondHatchStepDesired);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(secondStepChestTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame()); //2.5
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(secondStepTime, pelvisZUpFrame, pelvisYawPitchRollSecondHatchStepDesired); //2.0
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = getPelvisHeightTrajectoryMessage(secondStepTime, hatchFrame, pelvisPositionInHatchFrameSecondHatchStepDesired); //1.0            
            FootstepDataListMessage footstepDataListMessage = getFootstepDataListMessage(RobotSide.LEFT, leftFootSwingGoalPoint, leftFootSwingWayPoint1, leftFootSwingWayPoint2);
            
            // Send commands
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
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, chestYawPitchRollResetDesired);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(realignTime, chestOrientationFrame.getQuaternion(), chestOrientationFrame.getReferenceFrame(), chestOrientationFrame.getReferenceFrame());
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = getPelvisOrientationTrajectoryMessage(realignTime, pelvisZUpFrame, pelvisYawPitchRollResetDesired); //1.0
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
            atlasPrimitiveActions.walkToLocationBehavior.setFootstepLength(0.20);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(targetPose);
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> straighten = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.STRAIGHTEN_UP,
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Pelvis
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

      statemachine.addStateWithDoneTransition(searchForHatch, WalkThroughHatchBehaviorState.INITIALIZE_TRAJECTORIES);
      statemachine.addState(initializeTrajectories);
      initializeTrajectories.addStateTransition(WalkThroughHatchBehaviorState.INITIALIZE_CONFIGURATION, hatchPossibleCondition);
      initializeTrajectories.addStateTransition(WalkThroughHatchBehaviorState.FAILED, hatchImpossibleCondition);
      statemachine.addStateWithDoneTransition(initializeConfiguration, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkToHatchFarAction, WalkThroughHatchBehaviorState.SETUP_FOR_HATCH_WALK);
      statemachine.addStateWithDoneTransition(setupForHatchWalk, WalkThroughHatchBehaviorState.WALKING_TO_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkToHatchNearAction, WalkThroughHatchBehaviorState.ADJUST_CHEST);
      statemachine.addStateWithDoneTransition(adjustChest, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchFirstStep, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION);
      statemachine.addStateWithDoneTransition(walkThroughHatchTransition, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_SECOND_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchSecondStep, WalkThroughHatchBehaviorState.REALIGN_ROBOT);
      statemachine.addStateWithDoneTransition(realign, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_NEAR);
      statemachine.addStateWithDoneTransition(walkFromHatchNearAction, WalkThroughHatchBehaviorState.STRAIGHTEN_UP);
      statemachine.addStateWithDoneTransition(straighten, WalkThroughHatchBehaviorState.WALKING_FROM_HATCH_FAR);
      statemachine.addStateWithDoneTransition(walkFromHatchFarAction, WalkThroughHatchBehaviorState.DONE);
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
   
   private void setRobotTrajectoriesBasedOnHatchDimensions(int absHatch)
   {  
      int relHatch = absHatch - 1;
      
      Hatch hatch = HatchEnvironment.getHatch(relHatch);      
      hatchFrameViz.update();

      PrintTools.info("Found hatch at: " + hatchFrame.getTransformToWorldFrame().getTranslationVector().toString());
      
      hatchWidth = hatch.getHatchWidth();
      hatchThickness = hatch.getHatchThickness();
      hatchLowerHeight = hatch.getHatchStepHeight();
      hatchUpperHeight = hatch.getHatchOpeningHeight();
      
      if(useSafetyMarginForHatch)
      {
         hatchUpperHeight = hatchUpperHeight - 0.05;
         hatchThickness = hatchThickness + 0.01;
      }
      
      rightBeforeHatchOffset.set(targetLocationHatchBeforeNear.getX(), 0.0, 0.0);
      rightAfterHatchOffset.set(-rightBeforeHatchOffset.getX() - 0.03, 0.03, 0.0); // -0.01 in x
      leftBeforeHatchOffset.set(targetLocationHatchBeforeNear.getX(), 0.0, 0.0);
      leftAfterHatchOffset.set(-leftBeforeHatchOffset.getX() - 0.03, 0.03, 0.0); // +0.03 in x (-0.01 in x)
      
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
      pelvisYawPitchRollInitializeDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollInitializeDesired[1] = Math.toRadians(0.0);
      pelvisYawPitchRollInitializeDesired[2] = Math.toRadians(0.0);
      
      pelvisPositionInHatchFrameInitializeDesired.set(0.0, 0.0, defaultUpperPelvisHeight);
      
      pelvisYawPitchRollSetupDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollSetupDesired[1] = Math.toRadians(-hatchLowerHeight*100.0); //TEST: was -15.0 for 0.15 height
      pelvisYawPitchRollSetupDesired[2] = Math.toRadians(0.0);
      
      double pelvisMovementInitializationZ = -0.215 + 0.0475 * hatchLowerHeight/0.05 + (hatchUpperHeight - 1.55);
      if(pelvisMovementInitializationZ > 0.0)
         pelvisMovementInitializationZ = 0.0;
      pelvisPositionInHatchFrameSetupDesired.set(0.0, 0.0, defaultLowerPelvisHeight + pelvisMovementInitializationZ);
//      pelvisPositionInHatchFrameSetupDesired.set(0.0, 0.0, defaultUpperPelvisHeight);
      
      pelvisYawPitchRollFirstHatchStepDesired[0] = Math.toRadians(7.0);
      pelvisYawPitchRollFirstHatchStepDesired[1] = Math.toRadians(-hatchLowerHeight*100.0); //5.0
      pelvisYawPitchRollFirstHatchStepDesired[2] = Math.toRadians(-7.0); // -7.0
      
      pelvisPositionInHatchFrameFirstHatchStepDesired.set(pelvisPositionInHatchFrameSetupDesired);
      pelvisPositionInHatchFrameFirstHatchStepDesired.add(0.0, 0.0, 0.02); //0.02 in z (0.05 in x)
      
      pelvisYawPitchRollTransitionDesired[0] = Math.toRadians(0.0); //0.0
      pelvisYawPitchRollTransitionDesired[1] = Math.toRadians(4.0 + 2.0 * hatchLowerHeight/0.05);
      pelvisYawPitchRollTransitionDesired[2] = Math.toRadians(5.0); // roll was 15.0 for 0.15 height

      pelvisPositionInHatchFrameTransitionDesired.set(pelvisPositionInHatchFrameFirstHatchStepDesired);
      pelvisPositionInHatchFrameTransitionDesired.add(0.0, 0.0, 0.04); // 0.04 height, 0.025 + 0.025*hatchLowerHeight/0.05 - 0.04 in x
   
      pelvisYawPitchRollSecondHatchStepDesired[0] = Math.toRadians(-8.0); // -5
      pelvisYawPitchRollSecondHatchStepDesired[1] = Math.toRadians(hatchLowerHeight*100.0);
      pelvisYawPitchRollSecondHatchStepDesired[2] = Math.toRadians(12.0); // 10
      
      pelvisPositionInHatchFrameSecondHatchStepDesired.set(pelvisPositionInHatchFrameTransitionDesired);
      pelvisPositionInHatchFrameSecondHatchStepDesired.add(0.0, 0.0, 0.0);    
      
      pelvisYawPitchRollResetDesired[0] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[1] = Math.toRadians(0.0);
      pelvisYawPitchRollResetDesired[2] = Math.toRadians(0.0);
      
      pelvisPositionInHatchFrameRealignDesired.set(pelvisPositionInHatchFrameFirstHatchStepDesired);
      
      pelvisPositionInHatchFrameStraightenDesired.set(0.0, 0.0, defaultUpperPelvisHeight);
   }
   
   private void setFootSwingGoalPointsBasedOnHatchDimensions()
   {
      setRightFootSwingGoalPointBasedOnHatchDimensions();
      setLeftFootSwingGoalPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingGoalPointBasedOnHatchDimensions()
   {
      rightFootSwingGoalPoint.setToZero();
      rightFootSwingGoalPoint.sub(rightBeforeHatchOffset);
      rightFootSwingGoalPoint.add(rightAfterHatchOffset);
      rightFootSwingGoalPoint.add(new Point3D(hatchThickness, 0.0, 0.0));
   }
   
   private void setLeftFootSwingGoalPointBasedOnHatchDimensions()
   {
      leftFootSwingGoalPoint.setToZero();
      leftFootSwingGoalPoint.sub(leftBeforeHatchOffset);
      leftFootSwingGoalPoint.add(leftAfterHatchOffset);
      leftFootSwingGoalPoint.add(new Point3D(hatchThickness, 0.0, 0.0));
   }
   
   private void setFootSwingWayPointsBasedOnHatchDimensions()
   {
      setRightFootSwingWayPointBasedOnHatchDimensions();
      setLeftFootSwingWayPointBasedOnHatchDimensions();
   }
   
   private void setRightFootSwingWayPointBasedOnHatchDimensions()
   {
      rightFootSwingWayPoint1.add(defaultRightFootSwingWayPoint1, new Point3D(0, -0.03, hatchLowerHeight));
      
      rightFootSwingWayPoint2.add(defaultRightFootSwingWayPoint2, new Point3D(0, -0.03, hatchLowerHeight));
      rightFootSwingWayPoint2.add(rightFootSwingGoalPoint);
      
      if(rightFootSwingWayPoint2.getZ() < minSwingWayPointHeight)
      {
         double deltaSwingHeight = minSwingWayPointHeight - rightFootSwingWayPoint2.getZ();
         rightFootSwingWayPoint1.setZ(rightFootSwingWayPoint1.getZ() + deltaSwingHeight);
         rightFootSwingWayPoint2.setZ(rightFootSwingWayPoint2.getZ() + deltaSwingHeight);
      }
   }
   
   private void setLeftFootSwingWayPointBasedOnHatchDimensions()
   {
      leftFootSwingWayPoint1.add(defaultLeftFootSwingWayPoint1, new Point3D(0, 0.00, hatchLowerHeight)); // 0.03 ... was -0.02 in z
      
      leftFootSwingWayPoint2.add(defaultLeftFootSwingWayPoint2, new Point3D(0, 0.05, hatchLowerHeight)); // 0.05 ... 0.03 in y
      leftFootSwingWayPoint2.add(leftFootSwingGoalPoint);
      
      if(leftFootSwingWayPoint2.getZ() < minSwingWayPointHeight)
      {
         double deltaSwingHeight = minSwingWayPointHeight - leftFootSwingWayPoint2.getZ();
         leftFootSwingWayPoint1.setZ(leftFootSwingWayPoint1.getZ() + deltaSwingHeight);
         leftFootSwingWayPoint2.setZ(leftFootSwingWayPoint2.getZ() + deltaSwingHeight);
      }
   }
   
   public ChestTrajectoryMessage getChestOrientationTrajectoryMessage(double trajectoryTime, ReferenceFrame referenceFrame, double[] chestOrientation)
   {
      FrameOrientation chestOrientationFrame = new FrameOrientation(referenceFrame, chestOrientation);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      chestOrientationFrame.changeFrame(worldFrame);
      return new ChestTrajectoryMessage(trajectoryTime, chestOrientationFrame.getQuaternion(), worldFrame, worldFrame); // TODO: find out why not working
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
   
   public FootstepDataListMessage getFootstepDataListMessage(RobotSide robotSide, Point3D footSwingGoalPoint, Point3D footSwingWayPoint1, Point3D footSwingWayPoint2)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      footsteps.setExecutionMode(ExecutionMode.OVERRIDE);
      footsteps.setDestination(PacketDestination.BROADCAST);
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FramePose stepPose = new FramePose(soleFrame, footSwingGoalPoint, new Quaternion());
      stepPose.changeFrame(worldFrame);
      
      FramePose wayPointPose1 = new FramePose(soleFrame, footSwingWayPoint1, new Quaternion());
      wayPointPose1.changeFrame(worldFrame);
      
      FramePose wayPointPose2 = new FramePose(soleFrame, footSwingWayPoint2, new Quaternion());
      wayPointPose2.changeFrame(worldFrame);
      
      FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, stepPose.getPosition(), stepPose.getOrientation());
      footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
      footstepData.setCustomPositionWaypoints(new Point3D[] {new Point3D(wayPointPose1.getX(), wayPointPose1.getY(), wayPointPose1.getZ()), new Point3D(wayPointPose2.getX(), wayPointPose2.getY(), wayPointPose2.getZ())});
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
      currentHatch = 1;
   }
   

}
