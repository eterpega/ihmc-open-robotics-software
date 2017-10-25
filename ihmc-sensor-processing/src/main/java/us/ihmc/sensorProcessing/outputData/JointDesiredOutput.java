package us.ihmc.sensorProcessing.outputData;

public class JointDesiredOutput implements JointDesiredOutputReadOnly
{
   private JointDesiredControlMode controlMode;

   private double desiredTorque = Double.NaN;
   private double desiredPosition = Double.NaN;
   private double desiredVelocity = Double.NaN;
   private double desiredAcceleration = Double.NaN;
   private boolean resetIntegrators = false;

   private double stiffness = Double.NaN;
   private double damping = Double.NaN;
   private double masterGain = Double.NaN;

   public JointDesiredOutput()
   {
      clear();
   }

   public void clear()
   {
      controlMode = null;
      desiredTorque = Double.NaN;
      desiredPosition = Double.NaN;
      desiredVelocity = Double.NaN;
      desiredAcceleration = Double.NaN;
      resetIntegrators = false;

      stiffness = Double.NaN;
      damping = Double.NaN;
      masterGain = Double.NaN;
   }

   public void set(JointDesiredOutputReadOnly other)
   {
      clear();
      if (other.hasControlMode())
         controlMode = other.getControlMode();
      if (other.hasDesiredTorque())
         desiredTorque = other.getDesiredTorque();
      if (other.hasDesiredPosition())
         desiredPosition = other.getDesiredPosition();
      if (other.hasDesiredVelocity())
         desiredVelocity = other.getDesiredVelocity();
      if (other.hasDesiredAcceleration())
         desiredAcceleration = other.getDesiredAcceleration();
      if (other.hasStiffness())
         stiffness = other.getStiffness();
      if (other.hasDamping())
         damping = other.getDamping();
      if (other.hasMasterGain())
         masterGain = other.getMasterGain();
      resetIntegrators = other.peekResetIntegratorsRequest();
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputReadOnly other)
   {
      if (!hasControlMode() && other.hasControlMode())
         controlMode = other.getControlMode();
      if (!hasDesiredTorque() && other.hasDesiredTorque())
         desiredTorque = other.getDesiredTorque();
      if (!hasDesiredPosition() && other.hasDesiredPosition())
         desiredPosition = other.getDesiredPosition();
      if (!hasDesiredVelocity() && other.hasDesiredVelocity())
         desiredVelocity = other.getDesiredVelocity();
      if (!hasDesiredAcceleration() && other.hasDesiredAcceleration())
         desiredAcceleration = other.getDesiredAcceleration();
      if (!hasStiffness() && other.hasStiffness())
         stiffness = other.getStiffness();
      if (!hasDamping() && other.hasDamping())
         damping = other.getDamping();
      if (!hasMasterGain() && other.hasMasterGain())
         masterGain = other.getMasterGain();
      if (!peekResetIntegratorsRequest())
         resetIntegrators = other.peekResetIntegratorsRequest();
   }

   public void setControlMode(JointDesiredControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   public void setDesiredTorque(double tau)
   {
      desiredTorque = tau;
   }

   public void setDesiredPosition(double q)
   {
      desiredPosition = q;
   }

   public void setDesiredVelocity(double qd)
   {
      desiredVelocity = qd;
   }

   public void setDesiredAcceleration(double qdd)
   {
      desiredAcceleration = qdd;
   }

   public void setResetIntegrators(boolean reset)
   {
      resetIntegrators = reset;
   }

   @Override
   public boolean hasControlMode()
   {
      return controlMode != null;
   }

   @Override
   public boolean hasDesiredTorque()
   {
      return !Double.isNaN(desiredTorque);
   }

   @Override
   public boolean hasDesiredPosition()
   {
      return !Double.isNaN(desiredPosition);
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return !Double.isNaN(desiredVelocity);
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return !Double.isNaN(desiredAcceleration);
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      if (!hasControlMode())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a control mode.");
      return controlMode;
   }

   @Override
   public double getDesiredTorque()
   {
      if (!hasDesiredTorque())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a desired torque.");
      return desiredTorque;
   }

   @Override
   public double getDesiredPosition()
   {
      if (!hasDesiredPosition())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a desired position.");
      return desiredPosition;
   }

   @Override
   public double getDesiredVelocity()
   {
      if (!hasDesiredVelocity())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a desired velocity.");
      return desiredVelocity;
   }

   @Override
   public double getDesiredAcceleration()
   {
      if (!hasDesiredAcceleration())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a desired acceleration.");
      return desiredAcceleration;
   }

   @Override
   public boolean pollResetIntegratorsRequest()
   {
      boolean resetIntegrators = this.resetIntegrators;
      this.resetIntegrators = false;
      return resetIntegrators;
   }

   @Override
   public boolean peekResetIntegratorsRequest()
   {
      return resetIntegrators;
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (hasControlMode())
         ret += "controlMode = " + getControlMode() + "\n";
      if (hasDesiredTorque())
         ret += "desiredTorque = " + getDesiredTorque() + "\n";
      if (hasDesiredPosition())
         ret += "desiredPosition = " + getDesiredPosition() + "\n";
      if (hasDesiredVelocity())
         ret += "desiredVelocity = " + getDesiredVelocity() + "\n";
      if (hasDesiredAcceleration())
         ret += "desiredAcceleration = " + getDesiredAcceleration() + "\n";
      if (hasStiffness())
         ret += "masterGain = " + getStiffness() + "\n";
      if (hasDamping())
         ret += "masterGain = " + getDamping() + "\n";
      if (hasMasterGain())
         ret += "masterGain = " + getMasterGain() + "\n";
      if (ret.isEmpty())
         ret += "Data holder is empty.";
      return ret;
   }

   @Override
   public boolean hasStiffness()
   {
      return !Double.isNaN(stiffness);
   }

   @Override
   public boolean hasDamping()
   {
      return !Double.isNaN(damping);
   }

   @Override
   public double getStiffness()
   {
      if (!hasStiffness())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have stiffness.");
      return stiffness;
   }

   @Override
   public double getDamping()
   {
      if (!hasDamping())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have damping.");
      return damping;
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness = stiffness;
   }

   public void setDamping(double damping)
   {
      this.damping = damping;
   }

   @Override
   public boolean hasMasterGain()
   {
      return !Double.isNaN(masterGain);
   }

   @Override
   public double getMasterGain()
   {
      if (!hasMasterGain())
         throw new RuntimeException("This " + getClass().getSimpleName() + " does not have a master gain.");
      return masterGain;
   }

   public void setMasterGain(double masterGain)
   {
      this.masterGain = masterGain;
   }
}