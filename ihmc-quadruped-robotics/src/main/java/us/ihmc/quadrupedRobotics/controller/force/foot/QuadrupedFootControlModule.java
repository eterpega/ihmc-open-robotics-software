package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFootControlModule
{
   // control variables
   private final YoVariableRegistry registry;
   private final YoQuadrupedTimedStep stepCommand;
   private final YoBoolean stepCommandIsValid;

   private final GenericStateMachine<QuadrupedFootStates, QuadrupedFootState> footStateMachine;

   private QuadrupedStepTransitionCallback stepTransitionCallback;

   public QuadrupedFootControlModule(QuadrupedFootStateMachineParameters parameters, RobotQuadrant robotQuadrant, QuadrupedSolePositionController solePositionController,
                                     YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      // control variables
      String prefix = robotQuadrant.getCamelCaseName();
      this.registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());
      this.stepCommand = new YoQuadrupedTimedStep(prefix + "StepCommand", registry);
      this.stepCommandIsValid = new YoBoolean(prefix + "StepCommandIsValid", registry);

      // state machine
      QuadrupedSupportState supportState = new QuadrupedSupportState(robotQuadrant, stepCommandIsValid, timestamp, stepCommand);
      QuadrupedSwingState swingState = new QuadrupedSwingState(robotQuadrant, solePositionController, stepCommandIsValid, timestamp, stepCommand, parameters, registry);

      supportState.setDefaultNextState(QuadrupedFootStates.SWING);
      swingState.setDefaultNextState(QuadrupedFootStates.SUPPORT);

      footStateMachine = new GenericStateMachine<>(prefix + "FootState", "", QuadrupedFootStates.class, timestamp, registry);
      footStateMachine.addState(supportState);
      footStateMachine.addState(swingState);
      footStateMachine.setCurrentState(QuadrupedFootStates.SUPPORT);
      stepTransitionCallback = null;

      parentRegistry.addChild(registry);
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      this.stepTransitionCallback = stepTransitionCallback;
   }

   public void attachStateChangedListener(StateChangedListener<QuadrupedFootStates> stateChangedListener)
   {
      footStateMachine.attachStateChangedListener(stateChangedListener);
   }

   public void reset()
   {
      stepCommandIsValid.set(false);
   }

   public void triggerStep(QuadrupedTimedStep stepCommand)
   {
      if (footStateMachine.getCurrentStateEnum() == QuadrupedFootStates.SUPPORT)
      {
         this.stepCommand.set(stepCommand);
         this.stepCommandIsValid.set(true);
      }
   }

   public void adjustStep(FramePoint3D newGoalPosition)
   {
      this.stepCommand.setGoalPosition(newGoalPosition);
   }

   public ContactState getContactState()
   {
      if (footStateMachine.getCurrentStateEnum() == QuadrupedFootStates.SUPPORT)
         return ContactState.IN_CONTACT;
      else
         return ContactState.NO_CONTACT;
   }

   public void compute(FrameVector3D soleForceCommand, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      // Update estimates.
      footStateMachine.getCurrentState().updateEstimates(taskSpaceEstimates);

      // Update foot state machine.
      footStateMachine.doAction();

      // Pack sole force command result.
      soleForceCommand.set(footStateMachine.getCurrentState().getSoleForceCommand());
   }
}
