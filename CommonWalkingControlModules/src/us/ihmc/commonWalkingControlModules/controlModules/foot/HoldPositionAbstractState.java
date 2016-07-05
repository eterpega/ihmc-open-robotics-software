package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public abstract class HoldPositionAbstractState extends AbstractFootControlState
{
   public HoldPositionAbstractState(FootControlHelper footControlHelper, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, footControlHelper, registry);
   }

   public abstract void setWeight(double weight);
   public abstract void setWeights(Vector3d angular, Vector3d linear);
   public abstract void setDoSmartHoldPosition(boolean doSmartHold);
   public abstract void doFootholdAdjustments(boolean doAdjustments);
}
