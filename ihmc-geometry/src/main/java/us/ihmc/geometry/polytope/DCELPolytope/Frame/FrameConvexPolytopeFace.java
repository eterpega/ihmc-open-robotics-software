package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.FramePolytopeHalfEdgeBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

public class FrameConvexPolytopeFace extends ConvexPolytopeFaceBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> implements FrameSimplex, FrameSupportingVertexHolder, ReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;
   private final FramePolytopeHalfEdgeBuilder halfEdgeBuilder = new FramePolytopeHalfEdgeBuilder(this);
   
   public FrameConvexPolytopeFace(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   protected FrameConvexPolytopeFace getThis()
   {
      return this;
   }
   
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   protected PolytopeHalfEdgeProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }
}
