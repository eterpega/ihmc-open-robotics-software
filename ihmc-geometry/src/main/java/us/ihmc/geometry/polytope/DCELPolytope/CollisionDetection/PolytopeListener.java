package us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;

public interface PolytopeListener
{
   void attachPolytope(ConvexPolytopeReadOnly polytopeToAttach);
   void updateAll();
   void updateEdges();
   void updateVertices();
   void updateFaces();
}
