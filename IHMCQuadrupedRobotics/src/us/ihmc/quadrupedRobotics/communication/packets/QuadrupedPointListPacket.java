package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;
import java.util.ArrayList;

/**
 * Created by seanmason on 6/20/16.
 */
public class QuadrupedPointListPacket extends Packet<QuadrupedPointListPacket>
{
   public QuadrantDependentList<ArrayList<Point3d>> quadrantPointList;

   public QuadrupedPointListPacket()
   {
      quadrantPointList = new QuadrantDependentList<>();
   }

   public QuadrupedPointListPacket(QuadrantDependentList<ArrayList<Point3d>> quadrantPointList){
      this.quadrantPointList = quadrantPointList;
   }

   @Override public boolean epsilonEquals(QuadrupedPointListPacket other, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < this.quadrantPointList.get(quadrant).size() ; i++)
         {
            output &= this.quadrantPointList.get(quadrant).get(i).epsilonEquals(other.quadrantPointList.get(quadrant).get(i), epsilon);
         }
      }
      return output;
   }

   public QuadrantDependentList<ArrayList<Point3d>> get(){
      return quadrantPointList;
   }
   public ArrayList<Point3d> getQuadrant(RobotQuadrant key){
      return quadrantPointList.get(key);
   }
   public Point3d getPoint(RobotQuadrant key, int i){
      return quadrantPointList.get(key).get(i);
   }
   public Point3d getEndPoint(RobotQuadrant key){
      return quadrantPointList.get(key).get(quadrantPointList.get(key).size()-1);
   }
}
