package us.ihmc.geometry.polytope;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FramePolytopeHalfEdge extends FrameGeometryObject<FramePolytopeHalfEdge, PolytopeHalfEdge> implements PolytopeHalfEdgeBasics
{
   private final PolytopeHalfEdge polytopeHalfEdge;
   
   public FramePolytopeHalfEdge(ReferenceFrame referenceFrame, PolytopeHalfEdge geometryObject)
   {
      super(referenceFrame, geometryObject);
      this.polytopeHalfEdge = geometryObject;
   }

   @Override
   public PolytopeHalfEdgeBasics getTwinHalfEdge()
   {
      return polytopeHalfEdge.getTwinHalfEdge();
   }

   @Override
   public FramePolytopeHalfEdge getNextHalfEdge()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FramePolytopeHalfEdge getPreviousHalfEdge()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FramePolytopeVertex getOriginVertex()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FramePolytopeVertex getDestinationVertex()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ConvexPolytopeFaceBasics getFace()
   {
      return polytopeHalfEdge.getFace();
   }

   @Override
   public double getX()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getY()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getZ()
   {
      // TODO Auto-generated method stub
      return 0;
   }
   
   
}
