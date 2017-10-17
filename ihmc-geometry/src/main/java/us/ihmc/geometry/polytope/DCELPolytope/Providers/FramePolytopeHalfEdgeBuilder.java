package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameSimplex;

public class FramePolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex>
{
   @Override
   public FramePolytopeHalfEdge getHalfEdge(FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      return new FramePolytopeHalfEdge(origin, destination);
   }

   @Override
   public FramePolytopeHalfEdge getHalfEdge()
   {
      return new FramePolytopeHalfEdge();
   }

}
