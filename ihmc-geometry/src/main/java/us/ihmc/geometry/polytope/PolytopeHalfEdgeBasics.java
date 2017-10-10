package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeHalfEdgeBasics extends Vector3DReadOnly, Simplex
{
   PolytopeHalfEdgeBasics getTwinHalfEdge();
   PolytopeHalfEdgeBasics getNextHalfEdge();
   PolytopeHalfEdgeBasics getPreviousHalfEdge();
   PolytopeVertexBasics getOriginVertex();
   PolytopeVertexBasics getDestinationVertex();
   ConvexPolytopeFaceBasics getFace();
}
