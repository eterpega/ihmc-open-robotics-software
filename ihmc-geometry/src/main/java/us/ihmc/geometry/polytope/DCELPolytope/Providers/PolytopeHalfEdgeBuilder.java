package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Simplex;

public class PolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex>
{

   @Override
   public PolytopeHalfEdge getHalfEdge(ExtendedPolytopeVertex origin, ExtendedPolytopeVertex destination)
   {
      return new PolytopeHalfEdge(origin, destination);
   }

   @Override
   public PolytopeHalfEdge getHalfEdge()
   {
      return new PolytopeHalfEdge();
   }
}
