package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.FrameConvexPolytopeFaceBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.FramePolytopeVertexBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

public class FrameConvexPolytope extends ConvexPolytopeBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameConvexPolytope, FrameSimplex> implements ReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;
   private final FrameConvexPolytopeFaceBuilder faceBuilder = new FrameConvexPolytopeFaceBuilder(this);
   private final FramePolytopeVertexBuilder vertexBuilder = new FramePolytopeVertexBuilder(this);

   public FrameConvexPolytope()
   {
      this.referenceFrame = ReferenceFrame.getWorldFrame();
   }
   
   public FrameConvexPolytope(ReferenceFrame frame, ExtendedConvexPolytope polytope)
   {
      this();
   }
   
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   protected PolytopeVertexProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> getVertexProvider()
   {
      return vertexBuilder;
   }

   @Override
   protected ConvexPolytopeFaceProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> getConvexFaceProvider()
   {
      return faceBuilder;
   }
}
