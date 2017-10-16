package us.ihmc.geometry.polytope;

import java.util.List;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeVertexBasics extends Point3DBasics, Simplex
{
   PolytopeHalfEdgeBasics getAssociatedEdge(int index);
   List<? extends PolytopeHalfEdgeBasics> getAssociatedEdges();
   double dot(Vector3DReadOnly vector);
}
