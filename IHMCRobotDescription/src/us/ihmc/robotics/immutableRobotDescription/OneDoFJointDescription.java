package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Derived;
import us.ihmc.robotics.Axis;

import javax.vecmath.Vector3d;

public abstract class OneDoFJointDescription extends JointDescription
{
   public abstract Vector3d getAxis();

   // TODO: move this elsewhere or remove if not used
   public static Vector3d convertAxisVector(Axis axis)
   {
      switch (axis)
      {
      case X:
         return new Vector3d(1.0, 0.0, 0.0);
      case Y:
         return new Vector3d(0.0, 1.0, 0.0);
      case Z:
         return new Vector3d(0.0, 0.0, 1.0);
      }
      return new Vector3d(1, 0, 0);
   }

   @Default
   public double getDamping() {
      return 0;
   }

   @Default
   public double getStiction() {
      return 0;
   }

   @Default
   public double getVelocityLimit() {
      return Double.POSITIVE_INFINITY;
   }

   @Default
   public double getVelocityDamping() {
      return 0;
   }

   // TODO: getLimitStops should return optional/nullable and this should go away
   public boolean containsLimitStops()
   {
      return getLimitStops() != null;
   }

   @Default
   public LimitStops getLimitStops() {
      return null;
   }

   // TODO: the following two methods do not really make sense as they default to a somewhat arbitrary zero value if limits are not present
   @Derived
   public double getLowerLimit() {
      return getLimitStops() == null ? 0 : getLimitStops().getqMin();
   }

   @Derived
   public double getUpperLimit() {
      return getLimitStops() == null ? 0 : getLimitStops().getqMax();
   }

   @Default
   public double getEffortLimit() {
      return Double.POSITIVE_INFINITY;
   }

   public static class LimitStops
   {
      private final double qMin, qMax, kLimit, bLimit;

      public LimitStops(double qMin, double qMax, double kLimit, double bLimit)
      {
         this.qMin = qMin;
         this.qMax = qMax;
         this.kLimit = kLimit;
         this.bLimit = bLimit;
      }

      public double getqMin()
      {
         return qMin;
      }

      public double getqMax()
      {
         return qMax;
      }

      public double getkLimit()
      {
         return kLimit;
      }

      public double getbLimit()
      {
         return bLimit;
      }

      public double[] getArray()
      {
         return new double[] {qMin, qMax, kLimit, bLimit};
      }
   }

   static abstract class Builder implements JointDescription.Builder {}
}
