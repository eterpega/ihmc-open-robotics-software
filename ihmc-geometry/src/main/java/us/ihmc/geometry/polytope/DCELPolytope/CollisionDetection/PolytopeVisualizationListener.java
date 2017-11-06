package us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection;

import java.awt.Color;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;

public interface PolytopeVisualizationListener extends PolytopeListener
{
   void setColor(Color color);

   void setHighlightColor(Color color);

   void highlightFaces(List<? extends ConvexPolytopeFaceReadOnly> faces);

   void highlightEdges(List<? extends PolytopeHalfEdgeReadOnly> edges);

   void highlightVertices(List<? extends PolytopeVertexReadOnly> vertices);
   
   @Override
   default void updateVisibleSilhouette(List<? extends PolytopeHalfEdgeReadOnly> visibleEdges)
   {
      highlightEdges(visibleEdges);
   }

   void highlightEdge(PolytopeHalfEdgeReadOnly edgeToHighlight);
   
   @Override
   default void udpateVisibleEdgeSeed(PolytopeHalfEdgeReadOnly visibleEdgeSeed)
   {
      highlightEdge(visibleEdgeSeed);
   }
}
