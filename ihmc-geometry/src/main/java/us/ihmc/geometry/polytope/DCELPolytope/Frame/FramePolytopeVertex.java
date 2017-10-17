package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

public class FramePolytopeVertex extends PolytopeVertexBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace, FrameSimplex> implements FrameSimplex, ReferenceFrameHolder
{
   public FramePolytopeVertex()
   {
      super(new FramePoint3D());
   }
   
   public FramePolytopeVertex(ReferenceFrame frame, Point3DReadOnly vertex)
   {
      super(new FramePoint3D(frame, vertex));
   }
   
   public FramePolytopeVertex(FramePoint3D vertex)
   {
      super(new FramePoint3D(vertex));
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return ((FramePoint3D)getPosition()).getReferenceFrame();
   }
   
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      ((FramePoint3D) getPosition()).changeFrame(referenceFrame);
   }
   
   public void setIncludingFrame(FramePolytopeVertex vertexToSet)
   {
      
   }

}
