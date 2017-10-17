package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Simplex;

public class ConvexPolytopeFaceBuilder implements ConvexPolytopeFaceProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex>
{

   @Override
   public ConvexPolytopeFace getFace()
   {
      return new ConvexPolytopeFace();
   }
}
