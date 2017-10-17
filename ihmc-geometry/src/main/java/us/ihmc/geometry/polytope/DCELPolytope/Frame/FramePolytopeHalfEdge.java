package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;

public class FramePolytopeHalfEdge extends PolytopeHalfEdgeBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex>
      implements FrameSimplex, ReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;

   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, null, null);
   }

   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame, FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      this(referenceFrame, origin, destination, null, null, null, null);
   }

   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame, FramePolytopeVertex originVertex, FramePolytopeVertex destinationVertex,
                                FramePolytopeHalfEdge twinHalfEdge, FramePolytopeHalfEdge nextHalfEdge, FramePolytopeHalfEdge previousHalfEdge,
                                FrameConvexPolytopeFace face)
   {
      this.referenceFrame = referenceFrame;
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinHalfEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   @Override
   public void setOriginVertex(FramePolytopeVertex originVertex)
   {
      if (originVertex != null)
         checkReferenceFrameMatch(originVertex);
      super.setOriginVertex(originVertex);
   }

   @Override
   public void setOriginVertexUnsafe(FramePolytopeVertex originVertex)
   {
      if (originVertex != null)
         checkReferenceFrameMatch(originVertex);
      super.setOriginVertexUnsafe(originVertex);
   }

   @Override
   public void setDestinationVertex(FramePolytopeVertex destinationVertex)
   {
      if (destinationVertex!= null)
         checkReferenceFrameMatch(destinationVertex);
      super.setDestinationVertex(destinationVertex);
   }

   @Override
   public void setDestinationVertexUnsafe(FramePolytopeVertex destinationVertex)
   {
      if (destinationVertex != null)
         checkReferenceFrameMatch(destinationVertex);
      super.setDestinationVertexUnsafe(destinationVertex);
   }

   @Override
   public void setTwinHalfEdge(FramePolytopeHalfEdge twinEdge)
   {
      if (twinEdge != null)
         checkReferenceFrameMatch(twinEdge);
      super.setTwinHalfEdge(twinEdge);
   }

   @Override
   public void setNextHalfEdge(FramePolytopeHalfEdge nextHalfEdge)
   {
      if (nextHalfEdge != null)
         checkReferenceFrameMatch(nextHalfEdge);
      super.setNextHalfEdge(nextHalfEdge);
   }

   @Override
   public void setPreviousHalfEdge(FramePolytopeHalfEdge previousHalfEdge)
   {
      if (previousHalfEdge != null)
         checkReferenceFrameMatch(previousHalfEdge);
      super.setPreviousHalfEdge(previousHalfEdge);
   }

   @Override
   public void setFace(FrameConvexPolytopeFace face)
   {
      if (face != null)
         checkReferenceFrameMatch(face);
      super.setFace(face);
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
      return null;
   }
}
