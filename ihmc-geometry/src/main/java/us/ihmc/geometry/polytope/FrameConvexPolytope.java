package us.ihmc.geometry.polytope;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FrameConvexPolytope extends FrameGeometryObject<FrameConvexPolytope, ExtendedConvexPolytope>
{
   ExtendedConvexPolytope convexPolytope;
   
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
}
