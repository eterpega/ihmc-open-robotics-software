package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private final RobotQuadrant robotQuadrant;

   private final QuadrupedTaskSpaceEstimates estimates;
   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep stepCommand;

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, YoBoolean stepCommandIsValid, YoDouble timestamp, YoQuadrupedTimedStep stepCommand)
   {
      super(QuadrupedFootStates.SUPPORT);

      this.robotQuadrant = robotQuadrant;
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = timestamp;
      this.stepCommand = stepCommand;
      this.estimates = new QuadrupedTaskSpaceEstimates();
   }

   @Override
   public void updateEstimates(QuadrupedTaskSpaceEstimates estimates)
   {
      this.estimates.set(estimates);
   }

   @Override
   public void doTransitionIntoAction()
   {
      soleForceCommand.setToZero();
   }

   @Override
   public boolean isDone()
   {
      if (stepCommandIsValid.getBooleanValue())
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCommand.getTimeInterval().getStartTime();
         double touchDownTime = stepCommand.getTimeInterval().getEndTime();

         // trigger swing phase
         if (currentTime >= liftOffTime && currentTime < touchDownTime)
            return true;
      }

      return false;
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }
}
