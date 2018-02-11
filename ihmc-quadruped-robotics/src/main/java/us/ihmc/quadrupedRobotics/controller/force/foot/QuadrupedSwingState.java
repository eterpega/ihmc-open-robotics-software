package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSwingState extends QuadrupedFootState
{
   private final RobotQuadrant robotQuadrant;
   private final ThreeDoFSwingFootTrajectory swingTrajectory;
   private final FramePoint3D goalPosition;
   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final QuadrupedFootStateMachineParameters parameters;
   private final QuadrupedTaskSpaceEstimates estimates;

   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep stepCommand;

   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedSolePositionController solePositionController, YoBoolean stepCommandIsValid, YoDouble timestamp, YoQuadrupedTimedStep stepCommand,
                              QuadrupedFootStateMachineParameters parameters, YoVariableRegistry registry)
   {
      super(QuadrupedFootStates.SWING);

      this.solePositionController = solePositionController;

      this.robotQuadrant = robotQuadrant;
      this.goalPosition = new FramePoint3D();
      this.swingTrajectory = new ThreeDoFSwingFootTrajectory(this.robotQuadrant.getPascalCaseName(), registry);
      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          parameters.getTouchdownTriggerWindowParameter());
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = timestamp;
      this.stepCommand = stepCommand;
      this.parameters = parameters;

      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);
      estimates = new QuadrupedTaskSpaceEstimates();
   }

   @Override
   public void updateEstimates(QuadrupedTaskSpaceEstimates estimates)
   {
      this.estimates.set(estimates);
   }

   @Override
   public void doTransitionIntoAction()
   {
      // initialize swing trajectory
      double groundClearance = stepCommand.getGroundClearance();
      TimeInterval timeInterval = stepCommand.getTimeInterval();
      stepCommand.getGoalPosition(goalPosition);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.add(0.0, 0.0, parameters.getStepGoalOffsetZParameter());
      FramePoint3D solePosition = estimates.getSolePosition(robotQuadrant);
      solePosition.changeFrame(goalPosition.getReferenceFrame());
      swingTrajectory.initializeTrajectory(solePosition, goalPosition, groundClearance, timeInterval);

      // initialize contact state and feedback gains
      solePositionController.reset();
      solePositionController.getGains().setProportionalGains(parameters.getSolePositionProportionalGainsParameter());
      solePositionController.getGains().setDerivativeGains(parameters.getSolePositionDerivativeGainsParameter());
      solePositionController.getGains()
                            .setIntegralGains(parameters.getSolePositionIntegralGainsParameter(), parameters.getSolePositionMaxIntegralErrorParameter());
      solePositionControllerSetpoints.initialize(estimates);

      touchdownTrigger.set(false);
   }

   @Override
   public boolean isDone()
   {
      double currentTime = timestamp.getDoubleValue();
      double touchDownTime = stepCommand.getTimeInterval().getEndTime();

      // Trigger support phase.
      if (currentTime >= touchDownTime)
         return true;

      return false;
   }

   @Override
   public void doAction()
   {
      double currentTime = timestamp.getDoubleValue();
      double touchDownTime = stepCommand.getTimeInterval().getEndTime();

      // Compute current goal position.
      stepCommand.getGoalPosition(goalPosition);
      goalPosition.changeFrame(ReferenceFrame.getWorldFrame());
      goalPosition.add(0.0, 0.0, parameters.getStepGoalOffsetZParameter());

      // Compute swing trajectory.
      if (touchDownTime - currentTime > parameters.getMinimumStepAdjustmentTimeParameter())
      {
         swingTrajectory.adjustTrajectory(goalPosition, currentTime);
      }
      swingTrajectory.computeTrajectory(currentTime);
      swingTrajectory.getPosition(solePositionControllerSetpoints.getSolePosition());

      // Detect early touch-down.
      FrameVector3D soleForceEstimate = estimates.getSoleVirtualForce(robotQuadrant);
      soleForceEstimate.changeFrame(ReferenceFrame.getWorldFrame());
      double pressureEstimate = -soleForceEstimate.getZ();
      double relativeTimeInSwing = currentTime - stepCommand.getTimeInterval().getStartTime();
      double normalizedTimeInSwing = relativeTimeInSwing / stepCommand.getTimeInterval().getDuration();
      if (normalizedTimeInSwing > 0.5)
      {
         touchdownTrigger.update(pressureEstimate > parameters.getTouchdownPressureLimitParameter());
      }

      // Compute sole force.
      if (touchdownTrigger.getBooleanValue())
      {
         double pressureLimit = parameters.getTouchdownPressureLimitParameter();
         soleForceCommand.changeFrame(ReferenceFrame.getWorldFrame());
         soleForceCommand.set(0, 0, -pressureLimit);
      }
      else
      {
         solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, estimates);
         soleForceCommand.changeFrame(ReferenceFrame.getWorldFrame());
      }

   }

   @Override
   public void doTransitionOutOfAction()
   {
      soleForceCommand.setToZero();
      stepCommandIsValid.set(false);
   }
}
