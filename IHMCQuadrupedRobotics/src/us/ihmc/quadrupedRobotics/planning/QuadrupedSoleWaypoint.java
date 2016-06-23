package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * Created by seanmason on 6/23/16.
 */
public class QuadrupedSoleWaypoint
{
   public QuadrantDependentList<ArrayList<Point3d>> quadrantSolePositionList;
   public QuadrantDependentList<ArrayList<Vector3d>> quadrantSoleVelocityList;
   public QuadrantDependentList<ArrayList<Double>> quadrantSoleTimingList;

   public QuadrupedSoleWaypoint()
   {
      quadrantSolePositionList = new QuadrantDependentList<>();
      quadrantSoleVelocityList = new QuadrantDependentList<>();
      quadrantSoleTimingList = new QuadrantDependentList<>();
   }

   public QuadrupedSoleWaypoint(QuadrantDependentList<ArrayList<Point3d>> quadrantSolePositionList,
         QuadrantDependentList<ArrayList<Vector3d>> quadrantSoleVelocityList, QuadrantDependentList<ArrayList<Double>> quadrantSoleTimingList)
   {
      this(); //not sure if this is correct
      this.quadrantSolePositionList = quadrantSolePositionList;
      this.quadrantSoleVelocityList = quadrantSoleVelocityList;
      this.quadrantSoleTimingList = quadrantSoleTimingList;
   }


// For now I simply make everything public. I can add getters and setters later if needed.

//   //Getters
//   public QuadrantDependentList<ArrayList<Point3d>> getQuadrantPositionList(RobotQuadrant key)
//   {
//      return quadrantSolePositionList;
//   }
//
//   public QuadrantDependentList<ArrayList<Vector3d>> getQuadrantVelocityList(RobotQuadrant key)
//   {
//      return quadrantSoleVelocityList;
//   }
//
//   public QuadrantDependentList<ArrayList<Double>> getQuadrantTimingList(RobotQuadrant key)
//   {
//      return quadrantSoleTimingList;
//   }
//
//   public ArrayList<Point3d> getPositionList(RobotQuadrant key)
//   {
//      return quadrantSolePositionList.get(key);
//   }
//
//   public ArrayList<Vector3d> getVelocityList(RobotQuadrant key)
//   {
//      return quadrantSoleVelocityList.get(key);
//   }
//
//   public ArrayList<Double> getTimingList(RobotQuadrant key)
//   {
//      return quadrantSoleTimingList.get(key);
//   }
//
//   //Setters
//   public void setQuadrantPositionList(QuadrantDependentList<ArrayList<Point3d>> input)
//   {
//      this.quadrantSolePositionList = input;
//   }
//
//   public void setQuadrantVelocityList(QuadrantDependentList<ArrayList<Vector3d>> input)
//   {
//      this.quadrantSoleVelocityList = input;
//   }
//
//   public void setQuadrantTimingList(QuadrantDependentList<ArrayList<Double>> input)
//   {
//      this.quadrantSoleTimingList = input;
//   }
}
