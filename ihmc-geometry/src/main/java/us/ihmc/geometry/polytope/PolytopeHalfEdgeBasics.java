package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeHalfEdgeBasics extends Vector3DReadOnly
{
   PolytopeHalfEdgeBasics getTwinHalfEdge();
   PolytopeHalfEdgeBasics getNextHalfEdge();
   PolytopeHalfEdgeBasics getPreviousHalfEdge();
   PolytopeVertexBasics getOriginVertex();
   PolytopeVertexBasics getDestinationVertex();
   ConvexPolytopeFaceBasics getFace();
   
}
