package us.ihmc.geometry.polytope;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

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
}
