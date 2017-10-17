package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;

public class FrameConvexPolytope extends FrameGeometryObject<FrameConvexPolytope, ExtendedConvexPolytope> implements FrameSupportingVertexHolder, FrameSimplex
{
   ExtendedConvexPolytope convexPolytope;
   Vector3D tempVector = new Vector3D();
   FramePoint3D framePoint = new FramePoint3D();
   
   public FrameConvexPolytope(ReferenceFrame referenceFrame, ExtendedConvexPolytope geometryObject)
   {
      super(referenceFrame, geometryObject);
      this.convexPolytope = geometryObject;
   }
   
   public void addVertex(FramePolytopeVertex vertexToAdd, double epsilon)
   {
      checkReferenceFrameMatch(vertexToAdd);
      convexPolytope.addVertex(vertexToAdd.getGeometryObject(), epsilon);
   }

   @Override
   public double getShortestDistanceTo(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return convexPolytope.getShortestDistanceTo(point);
   }

   @Override
   public void getSupportVectorJacobianTo(FramePoint3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      checkReferenceFrameMatch(point);
      convexPolytope.getSupportVectorJacobianTo(point, jacobianToPack);
   }

   @Override
   public FrameSimplex getSmallestSimplexMemberReference(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return (FrameSimplex) convexPolytope.getSmallestSimplexMemberReference(point);
   }

   @Override
   public void getSupportVectorDirectionTo(FramePoint3DReadOnly point, FrameVector3D supportVectorToPack)
   {
      checkReferenceFrameMatch(point);
      convexPolytope.getSupportVectorDirectionTo(point, tempVector);
      supportVectorToPack.setIncludingFrame(getReferenceFrame(), tempVector);
   }

   @Override
   public FramePoint3D getSupportingVertex(FrameVector3D supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      Point3D tempPoint = convexPolytope.getSupportingVertex(supportDirection.getVector());
      framePoint.setIncludingFrame(getReferenceFrame(), tempPoint);
      return framePoint;
   }
}
