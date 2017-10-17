package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameSimplex;

public class FramePolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex>
{
   private final ReferenceFrameHolder referenceFrameHolder;
   
   public FramePolytopeHalfEdgeBuilder(ReferenceFrameHolder referenceFrameHolder)
   {
      this.referenceFrameHolder = referenceFrameHolder;
   }
   
   @Override
   public FramePolytopeHalfEdge getHalfEdge(FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      return new FramePolytopeHalfEdge(referenceFrameHolder.getReferenceFrame(), origin, destination);
   }

   @Override
   public FramePolytopeHalfEdge getHalfEdge()
   {
      return new FramePolytopeHalfEdge(referenceFrameHolder.getReferenceFrame());
   }

}
