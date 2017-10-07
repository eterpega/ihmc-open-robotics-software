package us.ihmc.geometry.polytope;

public interface PolytopeHalfEdgeBasics
{
   PolytopeHalfEdgeBasics getTwinHalfEdge();
   PolytopeHalfEdgeBasics getNextHalfEdge();
   PolytopeHalfEdgeBasics getPreviousHalfEdge();
   PolytopeVertexBasics getOriginVertex();
   PolytopeVertexBasics getDestinationVertex();
   ConvexPolytopeFaceBasics getFace();
   
}
