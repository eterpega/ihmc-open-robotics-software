package us.ihmc.manipulation.planning.collisionAvoidance;

import java.util.List;

import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class RobotCollisionBoxProvider
{
   public static List<ExtendedConvexPolytope> getCollisionMesh(RobotDescription robotDescription)
   {
      throw new RuntimeException("Not implmented");
   }
   
   public static ExtendedConvexPolytope getCollisionMesh()
   {
      return null;
   }
}
