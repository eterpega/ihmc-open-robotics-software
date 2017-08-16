package us.ihmc.atlas.atlasRosControl.sliderBoardControl;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.EffortJointHandle;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
class EffortJointHolder extends AtlasSliderBoardJointHolder
{
   private final EffortJointHandle handle;

   public EffortJointHolder(AtlasRosControlSliderBoard atlasRosControlSliderBoard, OneDoFJoint joint, EffortJointHandle handle,
         YoVariableRegistry parentRegistry, double dt)
   {
      super(atlasRosControlSliderBoard, joint, parentRegistry, dt);
      this.handle = handle;

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      joint.setQ(handle.getPosition());
      joint.setQd(handle.getVelocity());
      bl_qd.update();
      joint.setTauMeasured(handle.getEffort());

      q.set(joint.getQ());
      qd.set(joint.getQd());
      tau.set(joint.getTauMeasured());

      double pdOutput = pdController.compute(q.getDoubleValue(), q_d.getDoubleValue(), qd.getDoubleValue(), qd_d.getDoubleValue());
      jointCommand_pd.set(pdOutput);
      tau_d.set(atlasRosControlSliderBoard.masterScaleFactor.getDoubleValue() * (jointCommand_pd.getDoubleValue() + jointCommand_function.getDoubleValue()) + tau_offset
            .getDoubleValue());

      handle.setDesiredEffort(tau_d.getDoubleValue());
   }
}
