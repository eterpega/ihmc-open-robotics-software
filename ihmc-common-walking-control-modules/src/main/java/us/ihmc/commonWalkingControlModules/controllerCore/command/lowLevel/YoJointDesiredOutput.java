package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoJointDesiredOutput implements JointDesiredOutputReadOnly
{
   private final YoEnum<JointDesiredControlMode> controlMode;

   private final YoDouble desiredTorque;
   private final YoDouble desiredPosition;
   private final YoDouble desiredVelocity;
   private final YoDouble desiredAcceleration;
   private final YoBoolean resetIntegrators;

   private final YoDouble stiffness;
   private final YoDouble damping;
   private final YoDouble masterGain;

   public YoJointDesiredOutput(String namePrefix, YoVariableRegistry registry, String suffixString)
   {
      namePrefix += "LowLevel";

      controlMode = new YoEnum<>(namePrefix + "ControlMode" + suffixString, registry, JointDesiredControlMode.class, true);
      desiredTorque = new YoDouble(namePrefix + "DesiredTorque" + suffixString, registry);
      desiredPosition = new YoDouble(namePrefix + "DesiredPosition" + suffixString, registry);
      desiredVelocity = new YoDouble(namePrefix + "DesiredVelocity" + suffixString, registry);
      desiredAcceleration = new YoDouble(namePrefix + "DesiredAcceleration" + suffixString, registry);
      resetIntegrators = new YoBoolean(namePrefix + "ResetIntegrators" + suffixString, registry);

      stiffness = new YoDouble(namePrefix + "Stiffness" + suffixString, registry);
      damping = new YoDouble(namePrefix + "Damping" + suffixString, registry);
      masterGain = new YoDouble(namePrefix + "MasterGain" + suffixString, registry);

      clear();
   }

   public void clear()
   {
      controlMode.set(null);
      desiredTorque.set(Double.NaN);
      desiredPosition.set(Double.NaN);
      desiredVelocity.set(Double.NaN);
      desiredAcceleration.set(Double.NaN);
      stiffness.set(Double.NaN);
      damping.set(Double.NaN);
      masterGain.set(Double.NaN);
      resetIntegrators.set(false);
   }

   public void set(JointDesiredOutputReadOnly other)
   {
      clear();
      if (other.hasControlMode())
         controlMode.set(other.getControlMode());
      if (other.hasDesiredTorque())
         desiredTorque.set(other.getDesiredTorque());
      if (other.hasDesiredPosition())
         desiredPosition.set(other.getDesiredPosition());
      if (other.hasDesiredVelocity())
         desiredVelocity.set(other.getDesiredVelocity());
      if (other.hasDesiredAcceleration())
         desiredAcceleration.set(other.getDesiredAcceleration());
      if (other.hasStiffness())
         stiffness.set(other.getStiffness());
      if (other.hasDamping())
         damping.set(other.getDamping());
      if (other.hasMasterGain())
         masterGain.set(other.getMasterGain());
      resetIntegrators.set(other.peekResetIntegratorsRequest());
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputReadOnly other)
   {
      if (!hasControlMode() && other.hasControlMode())
         controlMode.set(other.getControlMode());
      if (!hasDesiredTorque() && other.hasDesiredTorque())
         desiredTorque.set(other.getDesiredTorque());
      if (!hasDesiredPosition() && other.hasDesiredPosition())
         desiredPosition.set(other.getDesiredPosition());
      if (!hasDesiredVelocity() && other.hasDesiredVelocity())
         desiredVelocity.set(other.getDesiredVelocity());
      if (!hasDesiredAcceleration() && other.hasDesiredAcceleration())
         desiredAcceleration.set(other.getDesiredAcceleration());
      if (!hasStiffness() && other.hasStiffness())
         stiffness.set(other.getStiffness());
      if (!hasDamping() && other.hasDamping())
         damping.set(other.getDamping());
      if (!hasMasterGain() && other.hasMasterGain())
         masterGain.set(other.getMasterGain());
      if (!peekResetIntegratorsRequest())
         resetIntegrators.set(other.peekResetIntegratorsRequest());
   }

   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   public void setDesiredTorque(double tau)
   {
      desiredTorque.set(tau);
   }

   public void setDesiredPosition(double q)
   {
      desiredPosition.set(q);
   }

   public void setDesiredVelocity(double qd)
   {
      desiredVelocity.set(qd);
   }

   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration.set(qdd);
   }

   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators.set(reset);
   }

   @Override
   public boolean hasControlMode()
   {
      return controlMode.getEnumValue() != null;
   }

   @Override
   public boolean hasDesiredTorque()
   {
      return !desiredTorque.isNaN();
   }

   @Override
   public boolean hasDesiredPosition()
   {
      return !desiredPosition.isNaN();
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !desiredVelocity.isNaN();
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !desiredAcceleration.isNaN();
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode.getEnumValue();
   }

   @Override
   public double getDesiredTorque()
   {
      return desiredTorque.getDoubleValue();
   }

   @Override
   public double getDesiredPosition()
   {
      return desiredPosition.getDoubleValue();
   }

   @Override
   public double getDesiredVelocity()
   {
      return desiredVelocity.getDoubleValue();
   }

   @Override
   public double getDesiredAcceleration()
   {
      return desiredAcceleration.getDoubleValue();
   }

   @Override
   public boolean pollResetIntegratorsRequest()
   {
      boolean request = resetIntegrators.getBooleanValue();
      resetIntegrators.set(false);
      return request;
   }

   @Override
   public boolean peekResetIntegratorsRequest()
   {
      return resetIntegrators.getBooleanValue();
   }

   @Override
   public String toString()
   {
      return getDescription();
   }

   @Override
   public boolean hasStiffness()
   {
      return !stiffness.isNaN();
   }

   @Override
   public boolean hasDamping()
   {
      return damping.isNaN();
   }

   @Override
   public double getStiffness()
   {
      return stiffness.getValue();
   }

   @Override
   public double getDamping()
   {
      return damping.getValue();
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness.set(stiffness);
   }

   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   @Override
   public boolean hasMasterGain()
   {
      return !masterGain.isNaN();
   }

   @Override
   public double getMasterGain()
   {
      return masterGain.getDoubleValue();
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain.set(masterGain);
   }
}
