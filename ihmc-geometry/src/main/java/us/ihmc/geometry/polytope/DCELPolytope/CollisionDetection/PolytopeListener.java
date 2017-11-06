package us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection;

import java.util.List;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;

public interface PolytopeListener
{
   void attachPolytope(ConvexPolytopeReadOnly polytopeToAttach);
   void updateAll();
   void updateEdges();
   void updateVertices();
   void updateFaces();
   void updateVisibleSilhouette(List<? extends PolytopeHalfEdgeReadOnly> visibleEdges);
   void udpateVisibleEdgeSeed(PolytopeHalfEdgeReadOnly visibleEdgeSeed);
}
