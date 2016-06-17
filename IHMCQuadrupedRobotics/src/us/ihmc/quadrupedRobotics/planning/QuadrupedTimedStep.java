package us.ihmc.quadrupedRobotics.planning;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedStep extends QuadrupedStep
{
   public static final int MAX_WAYPOINTS = 5;

   /**
    * The relative time interval of the swing phase (with respect to the start of the previous swing phase).
    */
   private final TimeInterval timeInterval;
   private boolean absolute = true;

   private List<QuadrupedStepWaypoint> waypoints; // reverse-sorted. 0 is the goal position, n is the first waypoint
   private int waypointCount;

   public QuadrupedTimedStep()
   {
      super();
      this.timeInterval = new TimeInterval(0.5, 1.0);
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, FramePoint goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3d goalPosition, double groundClearance, TimeInterval timeInterval)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public QuadrupedTimedStep(RobotQuadrant robotQuadrant, Point3d goalPosition, double groundClearance, TimeInterval timeInterval, boolean absolute)
   {
      super(robotQuadrant, goalPosition, groundClearance);
      this.timeInterval = new TimeInterval(timeInterval);
      this.absolute = absolute;
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public QuadrupedTimedStep(QuadrupedStep quadrupedStep, TimeInterval timeInterval)
   {
      super(quadrupedStep);
      this.timeInterval = new TimeInterval(timeInterval);
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public QuadrupedTimedStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      super(quadrupedTimedStep);
      this.timeInterval = new TimeInterval(quadrupedTimedStep.timeInterval);
      this.waypoints = new ArrayList<>(MAX_WAYPOINTS);
      for(int i = 0; i < MAX_WAYPOINTS; i++)
      {
         this.waypoints.add(new QuadrupedStepWaypoint(0.0, new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 0.0)));
      }
      this.waypointCount = 0;
   }

   public void set(QuadrupedTimedStep quadrupedTimedStep)
   {
      super.set(quadrupedTimedStep);
      this.timeInterval.set(quadrupedTimedStep.timeInterval);
      for (int i = 0; i < MAX_WAYPOINTS; i++)
      {
         if (i < quadrupedTimedStep.waypointCount)
         {
            waypoints.get(i).set(new QuadrupedStepWaypoint(quadrupedTimedStep.waypoints.get(i)));
         }
         else
         {
            waypoints.get(i).timeInStep = 0.0;
            waypoints.get(i).position.set(0.0, 0.0, 0.0);
            waypoints.get(i).velocity.set(0.0, 0.0, 0.0);
         }
      }
      this.waypointCount = quadrupedTimedStep.waypointCount;
   }

   public void get(QuadrupedTimedStep quadrupedTimedStep)
   {
      quadrupedTimedStep.set(this);
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public void getTimeInterval(TimeInterval timeInterval)
   {
      timeInterval.get(this.timeInterval);
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public boolean isAbsolute()
   {
      return absolute;
   }

   public void setAbsolute(boolean absolute)
   {
      this.absolute = absolute;
   }

   public void addWaypoint(QuadrupedStepWaypoint waypoint)
   {
      if (waypointCount + 1 > MAX_WAYPOINTS)
         throw new IndexOutOfBoundsException("too many waypoints: " + waypointCount + 1);

      waypoints.get(waypointCount).set(waypoint);
      waypointCount++;
   }

   public List<QuadrupedStepWaypoint> getWaypoints()
   {
      return waypoints;
   }

   public int getWaypointCount()
   {
      return waypointCount;
   }

   public boolean epsilonEquals(QuadrupedTimedStep other, double epsilon)
   {
      return super.epsilonEquals(other, epsilon) && this.timeInterval.epsilonEquals(other.timeInterval, epsilon);
   }

   @Override public String toString()
   {
      String string = super.toString();
      string += "\nstartTime: " + timeInterval.getStartTime();
      string += "\nendTime: " + timeInterval.getEndTime();
      return string;
   }
}

