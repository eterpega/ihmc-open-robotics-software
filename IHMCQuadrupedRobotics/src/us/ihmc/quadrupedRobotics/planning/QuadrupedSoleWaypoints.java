package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

public class QuadrupedSoleWaypoints extends QuadrantDependentList<ArrayList<SoleWaypoint>>
{

   public QuadrupedSoleWaypoints(){
      super();
   }
   public QuadrupedSoleWaypoints(QuadrupedSoleWaypoints other)
   {
      set(other);
   }

   public void set(QuadrupedSoleWaypoints other)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (other.isValid())
         {
            ArrayList<SoleWaypoint> soleWaypoints = new ArrayList<>();
            for (int i = 0; i < other.get(quadrant).size(); ++i)
            {
               soleWaypoints.add(new SoleWaypoint(other.get(quadrant).get(i)));
            }
            set(quadrant, soleWaypoints);
         }
      }
   }

   public boolean epsilonEquals(QuadrupedSoleWaypoints other, double epsilon)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (size(quadrant) != other.size(quadrant))
         {
            return false;
         }
         for (int i = 0; i < size(quadrant); ++i)
         {
            if (!this.get(quadrant).get(i).epsilonEquals(other.get(quadrant).get(i), epsilon));
            {
               return false;
            }
         }
      }
      return true;
   }

   public boolean hasNull()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         if (this.get(quadrant) == null)
         {
            return true;
         }
         for (int i = 0; i < this.get(quadrant).size(); ++i)
         {
            if (this.get(quadrant).get(i).hasNull())
            {
               return true;
            }
         }
      }
      return false;
   }

   public QuadrantDependentList<ArrayList<SoleWaypoint>> get()
   {
      return this;
   }

   public int size(RobotQuadrant key)
   {
      return this.get(key).size();
   }

   //todo: fill in with other checks
   public boolean isValid()
   {
      return !hasNull();
   }
}
