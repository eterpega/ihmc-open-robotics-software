package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value;
import us.ihmc.robotics.Axis;

import javax.vecmath.Vector3d;

public abstract class OneDoFJointDescription implements JointDescription
{
   public abstract Axis getAxis();

   @Value.Lazy
   public Vector3d getAxisVector() {
      switch (getAxis())
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

   public abstract double getDamping();

   public abstract double getStiction();

   public abstract double getVelocityLimit();

   public abstract double getVelocityDamping();

   public abstract Vector3d getJointAxis();

   public boolean containsLimitStops()
   {
      return getLimitStops() != null;
   }

   public abstract LimitStops getLimitStops();

   public abstract double getLowerLimit();

   public abstract double getUpperLimit();

   public abstract double getEffortLimit();

   public static class LimitStops {
      private final double qMin, qMax, kLimit, bLimit;

      public LimitStops(double qMin, double qMax, double kLimit, double bLimit) {
         this.qMin = qMin;
         this.qMax = qMax;
         this.kLimit = kLimit;
         this.bLimit = bLimit;
      }

      public double getqMin() {
         return qMin;
      }

      public double getqMax() {
         return qMax;
      }

      public double getkLimit() {
         return kLimit;
      }

      public double getbLimit() {
         return bLimit;
      }

      public double[] getArray() {
         return new double[] { qMin, qMax, kLimit, bLimit };
      }
   }
}
