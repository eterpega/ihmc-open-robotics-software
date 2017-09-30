package us.ihmc.geometry.polytope;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FramePolytopeVertex extends FrameGeometryObject<FramePolytopeVertex, PolytopeVertex>
{
   public FramePolytopeVertex(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      super(referenceFrame, new PolytopeVertex(x,y,z));
   }

}
