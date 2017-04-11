package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughHatchBehavior.WalkThroughHatchBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
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
      SET_UP_ROBOT_FOR_HATCH_WALK,
      WAITING_FOR_USER_CONFIRMATION,
      WALK_THROUGH_HATCH,
      CLEAR_HATCH_AREA,
      RESET_ROBOT,
      FAILED,
      DONE
   }

   private Vector3D32 hatchOffsetPoint1 = new Vector3D32(-0.21f - 0.5f, -0.08f + 0.05f, 0f);
   private Vector3D32 hatchOffsetPoint2 = new Vector3D32(-0.21f - 0.30f, -0.08f + 0.05f, 0f);
   
   private Vector3D32 hatchOffsetPoint3 = new Vector3D32(0.7f, -0.08f, 0f);
   private Vector3D32 hatchOffsetPoint4 = new Vector3D32(1.5f, -0.08f, 0f);
   
   private final HumanoidReferenceFrames referenceFrames;


   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;

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
      
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(yoTime, communicationBridge, atlasPrimitiveActions);
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
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior, atlasPrimitiveActions.pelvisTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            // HANDS
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
            
            // ARMS
            double[] leftArmPose = new double[] {-1.57, -0.51, 0.0, 2.0, 0.0, 0.0, 0.0};
            double[] rightArmPose = new double[] {1.57, 0.51, 0.0, -2.0, 0.0, 0.0, 0.0};

            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);
            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
            
            // CHEST
            ReferenceFrame chestFrame = referenceFrames.getChestFrame();
            FrameOrientation chestOrientation = new FrameOrientation(chestFrame, 0.0, Math.toRadians(15.0), 0.0);
            Quaternion chestOrientationWorld = new Quaternion();
            chestOrientation.getQuaternion(chestOrientationWorld);
            
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(1, chestOrientationWorld, chestFrame, referenceFrames.getPelvisFrame());
         
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            
            // PELVIS
            ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
            FramePose pelvisPose = new FramePose(pelvisFrame);
            
            AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); //TEST: was 0.00
            Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
            
            pelvisPose.setOrientation(pelvisOrientation);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            Point3D pelvisPositionWorld = new Point3D();
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisPose.getPose(pelvisPositionWorld, pelvisOrientationWorld);
            
            pelvisPositionWorld.setZ(pelvisPositionWorld.getZ() - 0.06); // TEST: was -0.06
            
            PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(1, pelvisPositionWorld, pelvisOrientationWorld);
            atlasPrimitiveActions.pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage);
            
            PrintTools.debug(this, "Done Initializing");
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> searchForHatch = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH, new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location
            
            PrintTools.debug(this, "Done Searching For Hatch");
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> walkToHatchAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALKING_TO_HATCH, walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint point1 = offsetPointFromHatch(hatchOffsetPoint1);
            FramePoint point2 = offsetPointFromHatch(hatchOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
            
            PrintTools.debug(this, "Done Walking To Hatch");
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> setUpForWalk = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.SET_UP_ROBOT_FOR_HATCH_WALK, atlasPrimitiveActions.footstepListBehavior,
            atlasPrimitiveActions.pelvisTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {            
            FootstepDataListMessage message = setUpFeetHatch();
            atlasPrimitiveActions.footstepListBehavior.set(message);
            
            // PELVIS
            ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
            FramePose pelvisPose = new FramePose(pelvisFrame);
            
            AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(-15.0));
            Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
            
            pelvisPose.setOrientation(pelvisOrientation);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            Point3D pelvisPositionWorld = new Point3D();
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisPose.getPose(pelvisPositionWorld, pelvisOrientationWorld);
            
            pelvisPositionWorld.setZ(pelvisPositionWorld.getZ() - 0.00); //TEST: was 0.00
            
            PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(1, pelvisPositionWorld, pelvisOrientationWorld);
            atlasPrimitiveActions.pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage);
            
            // CHEST
            ReferenceFrame chestFrame = referenceFrames.getChestFrame();
            FramePose chestPose = new FramePose(chestFrame);
            
            AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(15.0));
            Quaternion chestOrientation = new Quaternion(chestOrientationAA);
            
            chestPose.setOrientation(chestOrientation);
            chestPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            Quaternion chestOrientationWorld = new Quaternion();
            chestPose.getPose(new Point3D(), chestOrientationWorld);
            
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(1, chestOrientationWorld, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
         
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            
            PrintTools.debug(this, "Done Setting Up For Hatch Walk");
         }
      };

      BehaviorAction<WalkThroughHatchBehaviorState> walkThroughHatch = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH, atlasPrimitiveActions.footstepListBehavior, 
            atlasPrimitiveActions.pelvisTrajectoryBehavior, atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
            
            // PELVIS
            ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
            FramePose pelvisPose = new FramePose(pelvisFrame);
            
            AxisAngle pelvisOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(10.0)); // changed from 10.0
            Quaternion pelvisOrientation = new Quaternion(pelvisOrientationAA);
            
            pelvisPose.setOrientation(pelvisOrientation);
            pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            Point3D pelvisPositionWorld = new Point3D();
            Quaternion pelvisOrientationWorld = new Quaternion();
            pelvisPose.getPose(pelvisPositionWorld, pelvisOrientationWorld);
            
            pelvisPositionWorld.setZ(pelvisPositionWorld.getZ() + 0.02);
                        
            PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(4, pelvisPositionWorld, pelvisOrientationWorld);
            atlasPrimitiveActions.pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage);
            
            // CHEST
            ReferenceFrame chestFrame = referenceFrames.getChestFrame();
            FramePose chestPose = new FramePose(chestFrame);
            
            AxisAngle chestOrientationAA = new AxisAngle(0.0, 1.0, 0.0, Math.toRadians(0.0)); // TODO: adapt this better to task!!!
            Quaternion chestOrientation = new Quaternion(chestOrientationAA);
            
            chestPose.setOrientation(chestOrientation);
            chestPose.changeFrame(ReferenceFrame.getWorldFrame());
            
            Quaternion chestOrientationWorld = new Quaternion();
            chestPose.getPose(new Point3D(), chestOrientationWorld);
            
            ChestTrajectoryMessage chestOrientationTrajectoryMessage = new ChestTrajectoryMessage(3, chestOrientationWorld, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationTrajectoryMessage);
            
            PrintTools.debug(this, "Done Walking Through Hatch");
         }
      };
      
      BehaviorAction<WalkThroughHatchBehaviorState> clearHatchAreaAction = new BehaviorAction<WalkThroughHatchBehaviorState>(
            WalkThroughHatchBehaviorState.CLEAR_HATCH_AREA, walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint point1 = offsetPointFromHatch(hatchOffsetPoint3);
            FramePoint point2 = offsetPointFromHatch(hatchOffsetPoint4);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
            
            PrintTools.debug(this, "Done Clearing Hatch");
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

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return walkToInteractableObjectBehavior.isDone() && !walkToInteractableObjectBehavior.succeded();
         }
      };
      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return walkToInteractableObjectBehavior.isDone() && walkToInteractableObjectBehavior.succeded();
         }
      };

      statemachine.addStateWithDoneTransition(setup, WalkThroughHatchBehaviorState.SEARCHING_FOR_HATCH);
      statemachine.addStateWithDoneTransition(searchForHatch, WalkThroughHatchBehaviorState.WALKING_TO_HATCH);
      statemachine.addState(walkToHatchAction);
      walkToHatchAction.addStateTransition(WalkThroughHatchBehaviorState.SET_UP_ROBOT_FOR_HATCH_WALK, planSuccededCondition);
      walkToHatchAction.addStateTransition(WalkThroughHatchBehaviorState.FAILED, planFailedCondition);
      statemachine.addStateWithDoneTransition(setUpForWalk, WalkThroughHatchBehaviorState.WALK_THROUGH_HATCH);
      statemachine.addStateWithDoneTransition(walkThroughHatch, WalkThroughHatchBehaviorState.CLEAR_HATCH_AREA);
      statemachine.addStateWithDoneTransition(clearHatchAreaAction, WalkThroughHatchBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(failedState, WalkThroughHatchBehaviorState.DONE);
      statemachine.addState(doneState);
      
      statemachine.setStartState(WalkThroughHatchBehaviorState.SETUP_ROBOT);

   }

   private FramePoint offsetPointFromHatch(Vector3D32 point)
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("hatchFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset()));
      
      PrintTools.debug(this, HatchEnvironment.getHatchFrameOffset().toString());

      FramePoint point1 = new FramePoint(hatchPose, point);
      return point1;
   }
   
   public FootstepDataListMessage setUpFeetHatch()
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("HatchReferenceFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset()));

      RobotSide startStep = RobotSide.LEFT;

      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21, -0.08 + 0.125 + 0.005, 0.0), new Quaternion());
      FootstepDataMessage fs2 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21, -0.08 - 0.125 + 0.005, 0.0), new Quaternion());
      
      message.add(fs1);
      message.add(fs2);

      return message;

   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame hatchPose = new PoseReferenceFrame("HatchReferenceFrame", ReferenceFrame.getWorldFrame());
      hatchPose.setPoseAndUpdate(new RigidBodyTransform(new Quaternion(), HatchEnvironment.getHatchFrameOffset()));

      RobotSide startStep = RobotSide.RIGHT;

//      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
//            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());
      
      FootstepDataListMessage message = new FootstepDataListMessage(1.2,
                                                                    atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21 + 0.55 - 0.03, -0.08 - 0.125 + 0.005, 0.0), new Quaternion());
      FootstepDataMessage fs2 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21 + 0.55 - 0.03, -0.08 + 0.125 +0.005, 0.0), new Quaternion());
      
      FootstepDataMessage fs3 = createRelativeFootStep(hatchPose, startStep, new Point3D(-0.21 + 0.55 + 0.25, -0.08 - 0.125 + 0.005, 0.0), new Quaternion());
      FootstepDataMessage fs4 = createRelativeFootStep(hatchPose, startStep.getOppositeSide(), new Point3D(-0.21 + 0.55 + 0.25, -0.08 + 0.125 + 0.005, 0.0), new Quaternion());
      
      fs1.setSwingHeight(0.23);
      fs2.setSwingHeight(0.23);

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

   @Override
   public void onBehaviorExited()
   {
      
   }



}
