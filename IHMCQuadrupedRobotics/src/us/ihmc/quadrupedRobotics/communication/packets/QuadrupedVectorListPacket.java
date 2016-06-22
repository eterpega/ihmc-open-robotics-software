package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Vector3d;
import java.util.ArrayList;


/**
 * Created by seanmason on 6/20/16.
 */
public class QuadrupedVectorListPacket extends Packet<QuadrupedVectorListPacket>
{
   public QuadrantDependentList<ArrayList<Vector3d>> quadrantVectorList;

   public QuadrupedVectorListPacket()
   {
      quadrantVectorList = new QuadrantDependentList<>();
   }

   public QuadrupedVectorListPacket(QuadrantDependentList<ArrayList<Vector3d>> quadrantVectorList){
      this.quadrantVectorList = quadrantVectorList;
   }

   @Override public boolean epsilonEquals(QuadrupedVectorListPacket other, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < this.quadrantVectorList.get(quadrant).size() ; i++)
         {
            output &= this.quadrantVectorList.get(quadrant).get(i).epsilonEquals(other.quadrantVectorList.get(quadrant).get(i), epsilon);
         }
      }
      return output;
   }

   public QuadrantDependentList<ArrayList<Vector3d>> get(){
      return quadrantVectorList;
   }
   public ArrayList<Vector3d> getQuadrant(RobotQuadrant key){
      return quadrantVectorList.get(key);
   }
   public Vector3d getVector(RobotQuadrant key, int i){
      return quadrantVectorList.get(key).get(i);
   }
   public Vector3d getEndVector(RobotQuadrant key){
      return quadrantVectorList.get(key).get(quadrantVectorList.get(key).size()-1);
   }

}
