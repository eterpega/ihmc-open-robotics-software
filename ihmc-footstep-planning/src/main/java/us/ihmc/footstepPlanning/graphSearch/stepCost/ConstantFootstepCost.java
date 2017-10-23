package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class ConstantFootstepCost implements FootstepCost
{
   private final double cost;

   public ConstantFootstepCost(double cost)
   {
      this.cost = cost;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode, RigidBodyTransform endSnapTransform)
   {
      return cost;
   }
}
