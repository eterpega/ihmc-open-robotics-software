package us.ihmc.quadrupedRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import java.util.ArrayList;

import static java.lang.Math.abs;

/**
 * Created by seanmason on 6/20/16.
 */
public class QuadrupedDoubleListPacket extends Packet<QuadrupedDoubleListPacket>
{
   public QuadrantDependentList<ArrayList<Double>> quadrantDoubleList;

   public QuadrupedDoubleListPacket()
   {
      quadrantDoubleList = new QuadrantDependentList<>();
   }

   public QuadrupedDoubleListPacket(QuadrantDependentList<ArrayList<Double>> quadrantDoubleList){
      this.quadrantDoubleList = quadrantDoubleList;
   }

   @Override public boolean epsilonEquals(QuadrupedDoubleListPacket other, double epsilon)
   {
      boolean output = true;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < this.quadrantDoubleList.get(quadrant).size() ; i++)
         {
            output &= abs(this.quadrantDoubleList.get(quadrant).get(i) - other.quadrantDoubleList.get(quadrant).get(i)) < epsilon;
         }
      }
      return output;
   }
   public QuadrantDependentList<ArrayList<Double>> get(){
      return quadrantDoubleList;
   }
   public ArrayList<Double> getQuadrant(RobotQuadrant key){
      return quadrantDoubleList.get(key);
   }
   public Double getDouble(RobotQuadrant key, int i){
      return quadrantDoubleList.get(key).get(i);
   }
   public Double getEndDouble(RobotQuadrant key){
      return quadrantDoubleList.get(key).get(quadrantDoubleList.get(key).size()-1);
   }



}
