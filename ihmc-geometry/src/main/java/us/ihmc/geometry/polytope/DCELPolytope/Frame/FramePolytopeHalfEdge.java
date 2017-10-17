package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;

public class FramePolytopeHalfEdge extends PolytopeHalfEdgeBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> implements FrameSimplex
{
   public FramePolytopeHalfEdge()
   {
      
   }

   public FramePolytopeHalfEdge(FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      origin.checkReferenceFrameMatch(destination);
   }

   @Override
   public Point3DReadOnly getSupportingVertex(Vector3D supportDirection)
   {
      // TODO Auto-generated method stub
      return null;
   }

//   @Override
//   public FrameSimplex getSmallestSimplexMemberReference(Point3DReadOnly point)
//   {
//      // TODO Auto-generated method stub
//      return null;
//   }

   @Override
   public FramePolytopeHalfEdge createTwinHalfEdge(ConvexPolytopeFace twinEdgeFace)
   {
      // TODO Auto-generated method stub
      return null;
   }
   
   @Override
   public FramePolytopeHalfEdge getThis()
   {
      return this;
   }
}
