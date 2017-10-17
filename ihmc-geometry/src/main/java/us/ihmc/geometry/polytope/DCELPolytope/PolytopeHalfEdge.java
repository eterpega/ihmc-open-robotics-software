package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;

/**
 * This class implements a doubly connected edge list (https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
 * for storing polytope information
 * A half edge is completely described by its origin, destination and twin edge
 * The face, previous half edge and next half edge are stored for readability of code and should not be used for any geometrical operations
 * An attempt is made to update the twin in case the edge is modified to ensure that the relation remains consistent
 * @author Apoorv S
 */
public class PolytopeHalfEdge extends PolytopeHalfEdgeBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> implements GeometryObject<PolytopeHalfEdge>, Simplex
{
   
   public PolytopeHalfEdge()
   {
      super();
   }

   public PolytopeHalfEdge(ExtendedPolytopeVertex origin, ExtendedPolytopeVertex destination)
   {
      super(origin, destination);
   }
   
   public PolytopeHalfEdge(ExtendedPolytopeVertex originVertex, ExtendedPolytopeVertex destinationVertex, PolytopeHalfEdge twinEdge, PolytopeHalfEdge nextHalfEdge, PolytopeHalfEdge previousHalfEdge, ConvexPolytopeFace face)
   {
      super(originVertex, destinationVertex, twinEdge, nextHalfEdge, previousHalfEdge, face);
   }

   public PolytopeHalfEdge(PolytopeHalfEdge twinEdge, ConvexPolytopeFace face)
   {
      super(twinEdge, face);
   }
   
   @Override
   public PolytopeHalfEdge getThis()
   {
      return this;
   }

   @Override
   public PolytopeHalfEdge createTwinHalfEdge(ConvexPolytopeFace twinEdgeFace)
   {
      // TODO Auto-generated method stub
      return null;
   }
}
