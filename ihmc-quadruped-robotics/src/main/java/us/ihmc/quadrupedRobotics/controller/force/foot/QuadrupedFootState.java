package us.ihmc.quadrupedRobotics.controller.force.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class QuadrupedFootState extends FinishableState<QuadrupedFootStates>
{
   protected final FrameVector3D soleForceCommand = new FrameVector3D();

   public QuadrupedFootState(QuadrupedFootStates stateEnum)
   {
      super(stateEnum);
   }

   public abstract void updateEstimates(QuadrupedTaskSpaceEstimates estimates);

   public FrameVector3DReadOnly getSoleForceCommand()
   {
      return soleForceCommand;
   }
}
