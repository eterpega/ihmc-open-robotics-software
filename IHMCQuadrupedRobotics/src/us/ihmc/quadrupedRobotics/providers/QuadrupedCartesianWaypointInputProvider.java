package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FramePoint;

import java.util.ArrayList;

/**
 * Created by seanmason on 6/17/16.
 */
public class QuadrupedCartesianWaypointInputProvider
{

   private ArrayList<Double> timeAtWayPointList;
   private ArrayList<FramePoint> waypointPositionList;
   private ArrayList<FrameVector> waypointVelocityList;
   private Integer numberOfWaypoints;
   QuadrupedCartesianWaypointInputProvider(ArrayList<Double> timeAtWayPointList, ArrayList<FramePoint> waypointPositionList,
         ArrayList<FrameVector> waypointVelocityList)
   {
      if (numberOfWaypoints <= 2)
      {
         throw new RuntimeException("Trajectory has too few waypoints (min of 2).");
      }
      if (timeAtWayPointList.size() != waypointPositionList.size() || timeAtWayPointList.size() != waypointVelocityList.size())
      {
         throw new RuntimeException("Trajectory sizes are inconsistent.");
      }

      this.timeAtWayPointList = timeAtWayPointList;
      this.waypointPositionList = waypointPositionList;
      this.waypointVelocityList = waypointVelocityList;
      numberOfWaypoints = timeAtWayPointList.size();

   }

   public ArrayList<Double> getTimeAtWayPointList()
   {
      return timeAtWayPointList;
   }

   public ArrayList<FramePoint> getWaypointPositionList()
   {
      return waypointPositionList;
   }

   public ArrayList<FrameVector> getWaypointVelocityList()
   {
      return waypointVelocityList;
   }

   public double getTimeAtWayPoint(int i)
   {
      return timeAtWayPointList.get(i);
   }

   public FramePoint getWaypointPosition(int i)
   {
      return waypointPositionList.get(i);
   }

   public FrameVector getWaypointVelocity(int i)
   {
      return waypointVelocityList.get(i);
   }

   public int getNumberOfWaypoints()
   {
      return numberOfWaypoints;
   }
}
