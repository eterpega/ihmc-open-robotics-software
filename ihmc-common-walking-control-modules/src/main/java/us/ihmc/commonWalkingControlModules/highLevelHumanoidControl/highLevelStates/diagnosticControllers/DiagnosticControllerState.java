package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public abstract class DiagnosticControllerState extends State<DiagnosticMode>
{

   public DiagnosticControllerState(DiagnosticMode diagnosticMode)
   {
      super(diagnosticMode);
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public abstract void initializeControlGains(JointDesiredOutputListReadOnly jointsSettings);

   public abstract JointDesiredOutputListReadOnly getOutputForLowLevelController();
}
