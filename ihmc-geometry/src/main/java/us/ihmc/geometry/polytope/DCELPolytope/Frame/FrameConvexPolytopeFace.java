package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.FramePolytopeHalfEdgeBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

public class FrameConvexPolytopeFace extends ConvexPolytopeFaceBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> implements FrameSimplex, FrameSupportingVertexHolder
{

   public FrameConvexPolytopeFace()
   {
      super(new FramePolytopeHalfEdgeBuilder());
   }

   @Override
   public FramePoint3D getSupportingVertex(FrameVector3D supportDirection)
   {
      return null;
   }

   @Override
   protected FrameConvexPolytopeFace getThis()
   {
      return this;
   }
}
