package us.ihmc.geometry.polytope;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameConvexPolytopeFace extends FrameGeometryObject<FrameConvexPolytopeFace, ConvexPolytopeFace> implements ConvexPolytopeFaceBasics
{
   ConvexPolytopeFace convexPolytopeFace; 
   
   public FrameConvexPolytopeFace(ReferenceFrame referenceFrame, ConvexPolytopeFace geometryObject)
   {
      super(referenceFrame, geometryObject);
      this.convexPolytopeFace = geometryObject;
   }
   
   public void addVertex(FramePolytopeVertex vertexToAdd)
   {
      checkReferenceFrameMatch(vertexToAdd);
      this.convexPolytopeFace.addVertex(vertexToAdd.getGeometryObject());
   }

   @Override
   public boolean isMarked()
   {
      return convexPolytopeFace.isMarked();
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
