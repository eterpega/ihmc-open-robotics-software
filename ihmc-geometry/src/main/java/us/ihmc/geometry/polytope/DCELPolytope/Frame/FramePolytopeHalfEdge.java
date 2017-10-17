package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.FramePolytopeHalfEdgeBuilder;

public class FramePolytopeHalfEdge extends PolytopeHalfEdgeBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> implements FrameSimplex, ReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;
   private final FramePolytopeHalfEdgeBuilder halfEdgeBuilder = new FramePolytopeHalfEdgeBuilder(this);

   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, null, null);
   }

   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame, FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      super(origin, destination);
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setOriginVertex(FramePolytopeVertex originVertex)
   {
      checkReferenceFrameMatch(originVertex);
      super.setOriginVertex(originVertex);
   }
   
   @Override
   public void setOriginVertexUnsafe(FramePolytopeVertex originVertex)
   {
      checkReferenceFrameMatch(originVertex);
      super.setOriginVertexUnsafe(originVertex);
   }
   
   @Override
   public void setDestinationVertex(FramePolytopeVertex destinationVertex)
   {
      checkReferenceFrameMatch(destinationVertex);
      super.setDestinationVertex(destinationVertex);
   }
   
   @Override
   public void setDestinationVertexUnsafe(FramePolytopeVertex destinationVertex)
   {
      checkReferenceFrameMatch(destinationVertex);
      super.setDestinationVertexUnsafe(destinationVertex);
   }
   
   @Override
   public FramePolytopeHalfEdge getThis()
   {
      return this;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public FramePolytopeHalfEdge createTwinHalfEdge(FrameConvexPolytopeFace twinEdgeFace)
   {
      // TODO Auto-generated method stub
      return null;
   }
   
}
