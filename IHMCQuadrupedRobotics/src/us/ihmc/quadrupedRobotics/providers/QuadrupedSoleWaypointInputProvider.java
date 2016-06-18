package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

/**
 * Created by seanmason on 6/17/16.
 */
public class QuadrupedSoleWaypointInputProvider
{

   private QuadrantDependentList<ArrayList<Double>> timeAtWayPointQuadrantList;
   private QuadrantDependentList<ArrayList<FramePoint>> waypointPositionQuadrantList;
   private QuadrantDependentList<ArrayList<FrameVector>> waypointVelocityQuadrantList;

//   private ArrayList<Double> timeAtWayPointList;
//   private ArrayList<FramePoint> waypointPositionList;
//   private ArrayList<FrameVector> waypointVelocityList;
   private Integer numberOfWaypoints;
   QuadrupedSoleWaypointInputProvider(QuadrantDependentList<ArrayList<Double>> timeAtWayPointQuadrantList, QuadrantDependentList<ArrayList<FramePoint>> waypointPositionQuadrantList,
         QuadrantDependentList<ArrayList<FrameVector>> waypointVelocityQuadrantList)
   {
      for(RobotQuadrant quadrant: RobotQuadrant.values()){
         if (numberOfWaypoints <= 2)
         {
            throw new RuntimeException("Trajectory has too few waypoints (min of 2).");
         }
         if (timeAtWayPointQuadrantList.get(quadrant).size() != waypointPositionQuadrantList.get(quadrant).size() || timeAtWayPointQuadrantList.get(quadrant).size() != waypointVelocityQuadrantList.get(quadrant).size())
         {
            throw new RuntimeException("Trajectory sizes are inconsistent.");
         }
      }

      numberOfWaypoints = timeAtWayPointQuadrantList.get(RobotQuadrant.FRONT_LEFT).size();
      this.timeAtWayPointQuadrantList = timeAtWayPointQuadrantList;
      this.waypointPositionQuadrantList = waypointPositionQuadrantList;
      this.waypointVelocityQuadrantList = waypointVelocityQuadrantList;
   }

   public ArrayList<Double> getTimeAtWayPointList(RobotQuadrant quadrant)
   {
      return timeAtWayPointQuadrantList.get(quadrant);
   }

   public ArrayList<FramePoint> getWaypointPositionList(RobotQuadrant quadrant)
   {
      return waypointPositionQuadrantList.get(quadrant);
   }

   public ArrayList<FrameVector> getWaypointVelocityList(RobotQuadrant quadrant)
   {
      return waypointVelocityQuadrantList.get(quadrant);
   }

   public double getTimeAtWayPoint(RobotQuadrant quadrant, int i)
   {
      return timeAtWayPointQuadrantList.get(quadrant).get(i);
   }

   public FramePoint getWaypointPosition(RobotQuadrant quadrant,int i)
   {
      return waypointPositionQuadrantList.get(quadrant).get(i);
   }

   public FrameVector getWaypointVelocity(RobotQuadrant quadrant, int i)
   {
      return waypointVelocityQuadrantList.get(quadrant).get(i);
   }

   public int getNumberOfWaypoints()
   {
      return numberOfWaypoints;
   }
}
