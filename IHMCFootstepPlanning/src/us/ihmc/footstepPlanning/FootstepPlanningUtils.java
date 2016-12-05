package us.ihmc.footstepPlanning;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FootstepPlanningUtils
{
   public static FootstepPlan createFootstepPlanFromEndNode(BipedalFootstepPlannerNode endNode)
   {
      FootstepPlan plan = new FootstepPlan();
      plan.clear();
      BipedalFootstepPlannerNode node = endNode;

      while (node != null)
      {
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         node.getSoleTransform(soleTransform);

         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
         plan.addFootstep(node.getRobotSide(), framePose);

         node = node.getParentNode();
      }

      plan.reverse();
      return plan;
   }
}
