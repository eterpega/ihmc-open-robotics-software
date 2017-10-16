package us.ihmc.geometry.polytope;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

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

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      // TODO Auto-generated method stub
   }

   @Override
   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      // TODO Auto-generated method stub
      return null;
   }
   
   
}
