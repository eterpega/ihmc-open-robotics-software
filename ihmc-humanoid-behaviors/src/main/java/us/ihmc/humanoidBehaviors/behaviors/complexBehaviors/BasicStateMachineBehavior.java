package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.BasicStateMachineBehavior.BasicStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotSide;

public class BasicStateMachineBehavior extends StateMachineBehavior<BasicStates>
{
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private BehaviorAction<BasicStates> walkToBallTaskAndHomeArm;

   public enum BasicStates
   {
      ENABLE_LIDAR, CLEAR_LIDAR, WALK_TO_LOCATION_AND_HOME_ARM, BEHAVIOR_COMPLETE
   }

   public BasicStateMachineBehavior(String name, YoDouble yoTime, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(name, BasicStates.class, yoTime, outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      setUpStateMachine();
   }

   private void setUpStateMachine()
   {
      BehaviorAction<BasicStates> enableLidarTask = new BehaviorAction<BasicStates>(BasicStates.ENABLE_LIDAR, atlasPrimitiveActions.enableLidarBehavior);

      BehaviorAction<BasicStates> clearLidarTask = new BehaviorAction<BasicStates>(BasicStates.CLEAR_LIDAR, atlasPrimitiveActions.clearLidarBehavior);

      walkToBallTaskAndHomeArm = new BehaviorAction<BasicStates>(BasicStates.WALK_TO_LOCATION_AND_HOME_ARM, atlasPrimitiveActions.walkToLocationBehavior,
            atlasPrimitiveActions.leftArmGoHomeBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            GoHomeMessage goHomeLeftArmMessage = HumanoidMessageTools.createGoHomeMessage(BodyPart.ARM, RobotSide.LEFT, 2);
            atlasPrimitiveActions.leftArmGoHomeBehavior.setInput(goHomeLeftArmMessage);
            FramePose2D poseToWalkTo = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0, 0), 0);
            atlasPrimitiveActions.walkToLocationBehavior.setTarget(poseToWalkTo);
         }
      };
      statemachine.addStateWithDoneTransition(enableLidarTask, BasicStates.CLEAR_LIDAR);
      statemachine.addStateWithDoneTransition(clearLidarTask, BasicStates.WALK_TO_LOCATION_AND_HOME_ARM);
      statemachine.addState(walkToBallTaskAndHomeArm);
      statemachine.setStartState(BasicStates.ENABLE_LIDAR);

   }

   @Override
   public void onBehaviorExited()
   {
   }

}
