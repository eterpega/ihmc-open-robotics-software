package us.ihmc.geometry.polytope;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FrameConvexPolytope extends FrameGeometryObject<FrameConvexPolytope, ConvexPolytope>
{
   ConvexPolytope convexPolytope;
   
   public FrameConvexPolytope(ReferenceFrame referenceFrame, ConvexPolytope geometryObject)
   {
      super(referenceFrame, geometryObject);
      this.convexPolytope = geometryObject;
   }
   
   public void addVertex(FramePolytopeVertex vertexToAdd)
   {
      checkReferenceFrameMatch(vertexToAdd);
      convexPolytope.addVertex(vertexToAdd.getGeometryObject());
   }
}
