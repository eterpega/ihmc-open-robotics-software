package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
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
   public void onEntry()
   {
      soleForceCommand.setToZero();
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      if (stepCommandIsValid.getBooleanValue())
      {
         double currentTime = timestamp.getDoubleValue();
         double liftOffTime = stepCommand.getTimeInterval().getStartTime();
         double touchDownTime = stepCommand.getTimeInterval().getEndTime();

         // trigger swing phase
         if (currentTime >= liftOffTime && currentTime < touchDownTime)
         {
            if (stepTransitionCallback != null)
            {
               stepTransitionCallback.onLiftOff(robotQuadrant);
            }
            return QuadrupedFootControlModule.FootEvent.TIMEOUT;
         }
      }

      return null;
   }

   @Override
   public void onExit()
   {
   }
}
