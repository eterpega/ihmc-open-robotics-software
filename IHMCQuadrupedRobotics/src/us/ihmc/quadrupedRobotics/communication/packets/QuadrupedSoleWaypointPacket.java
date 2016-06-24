package us.ihmc.quadrupedRobotics.communication.packets;

import com.google.common.math.DoubleMath;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypoint;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * Created by seanmason on 6/20/16.
 */

public class QuadrupedSoleWaypointPacket extends Packet<QuadrupedSoleWaypointPacket>
{

   private QuadrupedSoleWaypoint quadrupedSoleWaypoint;

   public QuadrupedSoleWaypointPacket()
   {
      this.quadrupedSoleWaypoint = new QuadrupedSoleWaypoint();
   }
   public QuadrupedSoleWaypointPacket(QuadrupedSoleWaypoint quadrupedSoleWaypoint)
   {
      this.quadrupedSoleWaypoint = new QuadrupedSoleWaypoint();
      this.quadrupedSoleWaypoint.set(quadrupedSoleWaypoint);
   }
   public QuadrupedSoleWaypointPacket(QuadrantDependentList<ArrayList<Point3d>> quadrantSolePositionList,
         QuadrantDependentList<ArrayList<Vector3d>> quadrantSoleVelocityList, QuadrantDependentList<ArrayList<Double>> quadrantSoleTimingList)
   {
      this.quadrupedSoleWaypoint = new QuadrupedSoleWaypoint(quadrantSolePositionList, quadrantSoleVelocityList, quadrantSoleTimingList);
   }

   //Todo: Check epsilon equals logic
   @Override public boolean epsilonEquals(QuadrupedSoleWaypointPacket other, double epsilon)
   {
      boolean output = true;
      output &= epsilonEqualsQuadrantPointList(other.quadrupedSoleWaypoint.quadrantSolePositionList, epsilon);
      output &= epsilonEqualsQuadrantVectorList(other.quadrupedSoleWaypoint.quadrantSoleVelocityList, epsilon);
      output &= epsilonEqualsQuadrantDoubleList(other.quadrupedSoleWaypoint.quadrantSoleTimingList, epsilon);
      return output;
   }

   public boolean epsilonEqualsQuadrantPointList(QuadrantDependentList<ArrayList<Point3d>> quadrantSolePointList, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(this.quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).size() != quadrantSolePointList.get(quadrant).size()){
            return false;
         }
         for (int i = 0; i < this.quadrupedSoleWaypoint.quadrantSolePositionList.get(quadrant).size(); i++)
         {
            output &= this.quadrupedSoleWaypoint.quadrantSolePositionList.get(quadrant).get(i).epsilonEquals(quadrantSolePointList.get(quadrant).get(i), epsilon);
         }
      }
      return output;
   }

   public boolean epsilonEqualsQuadrantVectorList(QuadrantDependentList<ArrayList<Vector3d>> quadrantSoleVectorList, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(this.quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).size() != quadrantSoleVectorList.get(quadrant).size()){
            return false;
         }
         for (int i = 0; i < this.quadrupedSoleWaypoint.quadrantSoleVelocityList.get(quadrant).size(); i++)
         {
            output &= this.quadrupedSoleWaypoint.quadrantSoleVelocityList.get(quadrant).get(i).epsilonEquals(quadrantSoleVectorList.get(quadrant).get(i), epsilon);
         }
      }
      return output;
   }

   public boolean epsilonEqualsQuadrantDoubleList(QuadrantDependentList<ArrayList<Double>> quadrupedSoleWaypoint, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(this.quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).size() != quadrupedSoleWaypoint.get(quadrant).size()){
            return false;
         }
         for (int i = 0; i < this.quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).size(); i++)
         {
            output &= DoubleMath.fuzzyEquals(this.quadrupedSoleWaypoint.quadrantSoleTimingList.get(quadrant).get(i), quadrupedSoleWaypoint.get(quadrant).get(i), epsilon);
         }
      }
      return output;
   }

   public QuadrupedSoleWaypoint get() {return quadrupedSoleWaypoint;}

}
