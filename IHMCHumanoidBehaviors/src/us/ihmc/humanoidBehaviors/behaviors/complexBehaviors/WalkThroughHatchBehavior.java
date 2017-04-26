package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughHatchBehavior.WalkThroughHatchBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
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
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkThroughHatchBehavior extends StateMachineBehavior<WalkThroughHatchBehaviorState>
{
   public enum WalkThroughHatchBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      SETUP_ROBOT_ARMS,
      SEARCHING_FOR_HATCH,
      WALKING_TO_HATCH,
      SET_UP_ROBOT_FOR_HATCH_WALK_NEAR,
      WALK_THROUGH_HATCH_FIRST_STEP,
      WALK_THROUGH_HATCH_TRANSITION,
      WALK_THROUGH_HATCH_SECOND_STEP,
//      CLEAR_HATCH_AREA,
      RESET_ROBOT,
      FAILED,
      DONE
   }

   private Vector3D32 hatchOffsetPoint1 = new Vector3D32(-0.21f - 0.6f, -0.08f, 0f);
   private Vector3D32 hatchOffsetPoint2 = new Vector3D32(-0.21f, -0.08f, 0f);
   
   private final HumanoidReferenceFrames referenceFrames;
   
   
   
   private final Point3D hatchFrameOffset = new Point3D(HatchEnvironment.getHatchFrameOffset(0));
   private double hatchWidth = HatchEnvironment.getHatchWidth(0);
   private double hatchThickness = HatchEnvironment.getHatchThickness(0);
   private double hatchLowerHeight = HatchEnvironment.getHatchLowerHeight(0);
   private double hatchUpperHeight = HatchEnvironment.getHatchUpperHeight(0);
   
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
   
      
   private final DoubleYoVariable swingTime = new DoubleYoVariable("BehaviorSwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable("BehaviorTransferTime", registry);


   private final WalkToLocationBehavior walkToLocationBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   RobotSide side = RobotSide.RIGHT;
   
   public WalkThroughHatchBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("walkThroughHatchBehavior", WalkThroughHatchBehaviorState.class, yoTime, communicationBridge);

      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      
      swingTime.set(1.2);
      transferTime.set(0.6);

      WalkingControllerParameters walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();
      walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      registry.addChild(walkToLocationBehavior.getYoVariableRegistry());
      
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for door and updating its location here
      super.doControl();
   }

   private void setupStateMachine()
   {
      BehaviorAction<WalkThroughHatchBehaviorState> setup = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_ROBOT,
            atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior, atlasPrimitiveActions.pelvisHeightTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            setRobotTrajectoriesBasedOnHatchDimensions(0);
            
            // Chest
            ReferenceFrame chestFrame = referenceFrames.getChestFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(chestFrame, 0.0, Math.toRadians(17.0), 0.0);
            Quaternion chestOrientationLocal = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationLocal);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationLocal, chestFrame, referenceFrames.getPelvisZUpFrame());
            
            // Pelvis
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawInitialization[0], pelvisRollPitchYawInitialization[1], pelvisRollPitchYawInitialization[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(1.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisMovementInitialization);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.0, pelvisGoalLocation.getZ());            
            
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            
            PrintTools.debug(this, "Done Setting Up For Hatch Walk");
            
//            // Send commands
//            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            
            PrintTools.debug(this, "Done Initializing");
            
//            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
//            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
//            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
//            Point3D pelvisGoalLocation = new Point3D();
//            pelvisPose.getPosition(pelvisGoalLocation);
//            PrintTools.debug(this, "setup pelvis = " + pelvisGoalLocation.toString());
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> setupRobotArms = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.SETUP_ROBOT_ARMS,
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
            double[] leftArmPose = new double[] {-1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0};
            double[] rightArmPose = new double[] {1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0};
            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftArmPose);

            // Send commands
            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
            
            PrintTools.debug(this, "Done Initializing Arms");
            
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            PrintTools.debug(this, "setupRobotArms pelvis = " + pelvisGoalLocation.toString());
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatch = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH, new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the hatch location, inform the UI of its location
            
            PrintTools.debug(this, "Done Searching For Hatch");
            
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            PrintTools.debug(this, "searchForHatch pelvis = " + pelvisGoalLocation.toString());
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> walkToHatchAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_TO_HATCH, walkToLocationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint point1 = offsetPointFromHatch(hatchOffsetPoint1);
            FramePoint point2 = offsetPointFromHatch(hatchOffsetPoint2);
            point2.changeFrame(ReferenceFrame.getWorldFrame());

            FramePose2d target = new FramePose2d();
            target.checkReferenceFrameMatch(point2);
            target.setX(point2.getX());
            target.setY(point2.getY());
            target.setYaw(0.0);
            walkToLocationBehavior.setTarget(target);
            
            PrintTools.debug(this, "Done Walking To Hatch Far");
            
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPosition(pelvisGoalLocation);
            PrintTools.debug(this, "walkToHatch pelvis = " + pelvisGoalLocation.toString());
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
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(-7.0), Math.toRadians(27.0), Math.toRadians(0.0));
            Quaternion chestOrientationPelvisZUp = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawFirstStepThroughHatch[0], pelvisRollPitchYawFirstStepThroughHatch[1], pelvisRollPitchYawFirstStepThroughHatch[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);            
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.5, pelvisOrientationWorld); // was 1.5
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisMovementFirstStepThroughHatch);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(3.0, pelvisGoalLocation.getZ());
            
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
            footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

            wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint1 = new Point3D();
            wayPointPose1.getPose(locationWayPoint1, new Quaternion());
            wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint2 = new Point3D();
            wayPointPose2.getPose(locationWayPoint2, new Quaternion());
            
            footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
            footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
            footsteps.add(footstepData);
                        
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footsteps);
                        
            PrintTools.debug(this, "Done First Step Through Hatch");
            
            FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
            pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisGoalLocation.setToZero();
            pelvisPose2.getPosition(pelvisGoalLocation);  
            PrintTools.debug(this, "firstStep pelvis = " + pelvisGoalLocation.toString());
         }
      };
      
      
      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatchTransition = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION, atlasPrimitiveActions.pelvisTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            // Chest
            ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(25.0), Math.toRadians(0.0));
            Quaternion chestOrientationPelvisZUp = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3.0, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawReconfiguration[0], pelvisRollPitchYawReconfiguration[1], pelvisRollPitchYawReconfiguration[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.setPosition(pelvisMovementReconfiguration);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(2.0, pelvisGoalLocation, pelvisOrientationWorld);
            
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage);
            
            PrintTools.debug(this, "Done Transition Through Hatch");
            
            FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
            pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisGoalLocation.setToZero();
            pelvisPose2.getPosition(pelvisGoalLocation);
            PrintTools.debug(this, "transition pelvis = " + pelvisGoalLocation.toString());
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
            FrameOrientation chestOrientationFrame = new FrameOrientation(pelvisZUpFrame, Math.toRadians(0.0), Math.toRadians(7.0), Math.toRadians(0.0));
            Quaternion chestOrientationPelvisZUp = new Quaternion();
            chestOrientationFrame.getQuaternion(chestOrientationPelvisZUp);
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(2.5, chestOrientationPelvisZUp, pelvisZUpFrame, pelvisZUpFrame);
            
            // Pelvis
            FrameOrientation pelvisOrientationFrame = new FrameOrientation(pelvisZUpFrame, pelvisRollPitchYawSecondStepThroughHatch[0], pelvisRollPitchYawSecondStepThroughHatch[1], pelvisRollPitchYawSecondStepThroughHatch[2]);
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisOrientationFrame.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisOrientationFrame.getQuaternion(pelvisOrientationWorld);
            PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage = new PelvisOrientationTrajectoryMessage(2.0, pelvisOrientationWorld);
            
            FramePose pelvisPose = new FramePose(pelvisZUpFrame);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D pelvisGoalLocation = new Point3D();
            pelvisPose.getPose(pelvisGoalLocation, new Quaternion());
            PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = new PelvisHeightTrajectoryMessage(1.0, pelvisGoalLocation.getZ());
            
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
            footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
            
            wayPointPose1.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint1 = new Point3D();
            wayPointPose1.getPose(locationWayPoint1, new Quaternion());
            wayPointPose2.changeFrame(ReferenceFrame.getWorldFrame());
            Point3D locationWayPoint2 = new Point3D();
            wayPointPose2.getPose(locationWayPoint2, new Quaternion());
            
            footstepData.setTrajectoryType(TrajectoryType.CUSTOM);
            footstepData.setTrajectoryWaypoints(new Point3D[] {locationWayPoint1, locationWayPoint2});
            footsteps.add(footstepData);
            
            // Send commands
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisOrientationTrajectoryBehavior.setInput(pelvisOrientationTrajectoryMessage);
            atlasPrimitiveActions.pelvisHeightTrajectoryBehavior.setInput(pelvisHeightTrajectoryMessage);
            atlasPrimitiveActions.footstepListBehavior.set(footsteps);
                        
            PrintTools.debug(this, "Done Second Step Through Hatch");
            
            FramePose pelvisPose2 = new FramePose(pelvisZUpFrame);
            pelvisPose2.changeFrame(ReferenceFrame.getWorldFrame());
            pelvisGoalLocation.setToZero();
            pelvisPose2.getPosition(pelvisGoalLocation);
            PrintTools.debug(this, "secondStep pelvis = " + pelvisGoalLocation.toString());
         }
      };
      
      
//      BehaviorAction<WalkThroughHatchBehaviorState> clearHatchAreaAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
//            WalkThroughHatchBehaviorState.CLEAR_HATCH_AREA, walkToInteractableObjectBehavior)
//      {
//         @Override
//         protected void setBehaviorInput()
//         {
//            FramePoint point1 = offsetPointFromHatch(hatchOffsetPoint3);
//            FramePoint point2 = offsetPointFromHatch(hatchOffsetPoint4);
//
//            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
//            
//            PrintTools.debug(this, "Done Clearing Hatch");
//         }
//      };

      BehaviorAction<WalkThroughHatchBehaviorState> failedState = new BehaviorAction<WalkThroughHatchBehaviorState>(WalkThroughHatchBehaviorState.FAILED,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking Through Hatch Failed");
            sendPacket(p1);
            
            PrintTools.debug(this, "Done Failing");
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
            
            PrintTools.debug(this, "Done Done-ing");
         }
      };

//      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
//      {
//         @Override
//         public boolean checkCondition()
//         {
//            return walkToLocationBehavior.isDone();
//         }
//      };
      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return walkToLocationBehavior.isDone();
         }
      };

      statemachine.addStateWithDoneTransition(setup, WalkThroughHatchBehaviorState.SETUP_ROBOT_ARMS);
      statemachine.addStateWithDoneTransition(setupRobotArms, WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH);
      
//      statemachine.addStateWithDoneTransition(searchForHatch, WalkThroughHatchBehaviorState.SET_UP_ROBOT_FOR_HATCH_WALK);
      statemachine.addStateWithDoneTransition(searchForHatch, WalkThroughHatchBehaviorState.WALKING_TO_HATCH);
      statemachine.addState(walkToHatchAction);
      walkToHatchAction.addStateTransition(WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP, planSuccededCondition);
//      walkToHatchAction.addStateTransition(WalkThroughHatchBehaviorState.FAILED, planFailedCondition);
//      statemachine.addStateWithDoneTransition(adjustStepPositionsFar, WalkThroughHatchBehaviorState.SET_UP_ROBOT_FOR_HATCH_WALK_FAR);
//      statemachine.addStateWithDoneTransition(setUpForWalkFar, WalkThroughHatchBehaviorState.ADJUST_STEP_POSITIONS_NEAR);
//      statemachine.addStateWithDoneTransition(adjustStepPositionsNear, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_FIRST_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchFirstStep, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_TRANSITION);
      statemachine.addStateWithDoneTransition(walkThroughHatchTransition, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH_SECOND_STEP);
      statemachine.addStateWithDoneTransition(walkThroughHatchSecondStep, WalkThroughHatchBehaviorState.DONE);
//      statemachine.addStateWithDoneTransition(clearHatchAreaAction, WalkThroughHatchBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(failedState, WalkThroughHatchBehaviorState.DONE);
      statemachine.addState(doneState);
      
      statemachine.setStartState(WalkThroughHatchBehaviorState.SETUP_ROBOT);

   }

   private FramePoint offsetPointFromHatch(Vector3D32 point)
   {

      PoseReferenceFrame hatchFrame = new PoseReferenceFrame("hatchFrame", ReferenceFrame.getWorldFrame());
      hatchFrame.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset(0)));
      
      PrintTools.debug(this, HatchEnvironment.getHatchFrameOffset(0).toString());

      FramePoint point1 = new FramePoint(hatchFrame, point);
      return point1;
   }
   
   public FootstepDataListMessage setUpFeetHatchFar()
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("HatchReferenceFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset(0)));

      RobotSide startStep = RobotSide.LEFT;

      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21 - 0.20, -0.08 + 0.16, 0.0), new Quaternion()); // was 0.125
      FootstepDataMessage fs2 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21 - 0.20, -0.08 - 0.16, 0.0), new Quaternion());
      
      message.add(fs1);
      message.add(fs2);

      return message;

   }
   
   public FootstepDataListMessage setUpFeetHatchNear()
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("HatchReferenceFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset(0)));

      RobotSide startStep = RobotSide.LEFT;

      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21, -0.08 + 0.16, 0.0), new Quaternion()); // was 0.125
      FootstepDataMessage fs2 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21, -0.08 - 0.16, 0.0), new Quaternion());
      
      message.add(fs1);
      message.add(fs2);

      return message;

   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("HatchReferenceFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset(0)));

      RobotSide startStep = RobotSide.RIGHT;

//      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
//            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());
      
      FootstepDataListMessage message = new FootstepDataListMessage(1.2,
                                                                    atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21 + 0.55 - 0.03, -0.08 - 0.125 + 0.005, 0.0), new Quaternion());
      FootstepDataMessage fs2 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21 + 0.55 - 0.03, -0.08 + 0.125 +0.005, 0.0), new Quaternion());
      
      FootstepDataMessage fs3 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21 + 0.55 + 0.25, -0.08 - 0.125 + 0.005, 0.0), new Quaternion());
      FootstepDataMessage fs4 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21 + 0.55 + 0.25, -0.08 + 0.125 + 0.005, 0.0), new Quaternion());
      
      fs1.setSwingHeight(0.25);
      fs2.setSwingHeight(0.25);

      message.add(fs1);
      message.add(fs2);
      message.add(fs3);
      message.add(fs4);

      return message;

   }

   private FootstepDataMessage createRelativeFootStep(PoseReferenceFrame frame, RobotSide side, Point3D location, Quaternion orientation)
   {

      FramePose pose = offsetPointFromFrameInWorldFrame(frame, location, orientation);
      FootstepDataMessage message = new FootstepDataMessage(side, pose.getFramePointCopy().getPoint(), pose.getFrameOrientationCopy().getQuaternion());
      return message;
   }

   private FramePose offsetPointFromFrameInWorldFrame(PoseReferenceFrame frame, Point3D point3d, Quaternion quat4d)
   {
      FramePoint point1 = new FramePoint(frame, point3d);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameOrientation orient = new FrameOrientation(frame, quat4d);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose pose = new FramePose(point1, orient);

      return pose;
   }
   
   
   
   private void setRobotTrajectoriesBasedOnHatchDimensions(int hatch)
   {
      hatchFrameOffset.set(HatchEnvironment.getHatchFrameOffset(hatch));
      hatchWidth = HatchEnvironment.getHatchWidth(hatch);
      hatchThickness = HatchEnvironment.getHatchThickness(hatch);
      hatchLowerHeight = HatchEnvironment.getHatchLowerHeight(hatch);
      hatchUpperHeight = HatchEnvironment.getHatchUpperHeight(hatch);
      
      rightBeforeHatchOffset.set(0.22, 0.0, 0.0);
      rightAfterHatchOffset.set(rightBeforeHatchOffset.getX() - 0.01, 0.03, 0.0);
      leftBeforeHatchOffset.set(0.22, 0.0, 0.0);
      leftAfterHatchOffset.set(leftBeforeHatchOffset.getX() + 0.03 - 0.01, 0.03, 0.0);
      
      setPelvisTrajectoriesBasedOnHatchDimensions();
      setFootSwingGoalPointsBasedOnHatchDimensions();
      setFootSwingWayPointsBasedOnHatchDimensions();
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

      pelvisMovementReconfiguration.set(0.025 + 0.025*hatchLowerHeight/0.05, 0.0, 0.02); // 0.02 height
   
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
   
   @Override
   public void onBehaviorExited()
   {
      
   }
   

}
