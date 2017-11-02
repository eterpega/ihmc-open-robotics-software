package us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;

public interface PolytopeListener
{
   void update(ConvexPolytopeReadOnly simplex);

   void blockWhenInControl();
}
