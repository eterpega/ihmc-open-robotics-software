package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface FrameSupportingVertexHolder
{
   FramePoint3D getSupportingVertex(FrameVector3D supportDirection);
}
