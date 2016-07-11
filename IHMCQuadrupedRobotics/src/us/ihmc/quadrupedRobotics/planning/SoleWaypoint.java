package us.ihmc.quadrupedRobotics.planning;

import com.google.common.math.DoubleMath;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class SoleWaypoint
{
   private Point3d position;
   private Vector3d velocity;
   private Double time;

   public SoleWaypoint()
   {
      position = new Point3d(0, 0, 0);
      velocity = new Vector3d(0, 0, 0);
      time = new Double(0);
   }

   public SoleWaypoint(SoleWaypoint other)
   {
      this.position = new Point3d(other.position);
      this.velocity = new Vector3d(other.velocity);
      this.time = new Double(other.time);
   }

   public SoleWaypoint(Point3d position, Vector3d velocity, Double time)
   {
      set(position, velocity, time);
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Vector3d getVelocity()
   {
      return velocity;
   }

   public Double getTime()
   {
      return time;
   }

   public void set(Point3d position, Vector3d velocity, Double time)
   {
      this.position = position;
      this.velocity = velocity;
      this.time = time;
   }

   public void setPosition(Point3d position)
   {
      this.position = position;
   }

   public void setVelocity(Vector3d velocity)
   {
      this.velocity = velocity;
   }

   public void setTime(Double time)
   {
      this.time = time;
   }

   public boolean epsilonEquals(SoleWaypoint other, double epsilon)
   {
      return position.epsilonEquals(other.position, epsilon) || velocity.epsilonEquals(other.velocity, epsilon) || DoubleMath
            .fuzzyEquals(time, other.time, epsilon);
   }

   public boolean hasNull()
   {
      return (position == null || velocity == null || time == null);
   }
}
