package us.ihmc.quadrupedRobotics.planning;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class QuadrupedStepWaypoint
{
   public double timeInStep;
   public Point3d position;
   public Vector3d velocity;

   public QuadrupedStepWaypoint()
   {
      this(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0));
   }

   public QuadrupedStepWaypoint(double timeInStep, Point3d position, Vector3d velocity)
   {
      this.timeInStep = timeInStep;
      this.position = position;
      this.velocity = velocity;
   }

   public QuadrupedStepWaypoint(QuadrupedStepWaypoint waypoint)
   {
      this.timeInStep = 0.0;
      this.position = new Point3d(0.0, 0.0, 0.0);
      this.velocity = new Vector3d(0.0, 0.0, 0.0);
      this.set(waypoint);
   }

   public void set(QuadrupedStepWaypoint waypoint)
   {
      this.timeInStep = waypoint.timeInStep;
      this.position.set(waypoint.position);
      this.velocity.set(waypoint.velocity);
   }

   @Override
   public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      QuadrupedStepWaypoint that = (QuadrupedStepWaypoint) o;

      if (Double.compare(that.timeInStep, timeInStep) != 0)
         return false;
      if (position != null ? !position.equals(that.position) : that.position != null)
         return false;
      return velocity != null ? velocity.equals(that.velocity) : that.velocity == null;
   }

   @Override
   public String toString()
   {
      return "QuadrupedStepWaypoint{" +
            "timeInStep=" + timeInStep +
            ", position=" + position +
            ", velocity=" + velocity +
            '}';
   }
}
