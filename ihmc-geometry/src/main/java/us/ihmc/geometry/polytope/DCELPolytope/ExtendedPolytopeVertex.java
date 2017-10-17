package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

/**
 * This class stores the location of a point which is the vertex of a polytope
 * A list of polytope edges originating from this vertex is also stored for ease of algorithm design 
 * Faces to which this vertex belongs can be accessed by iterating through the list of edges
 * 
 * @author Apoorv S
 *
 */
public class ExtendedPolytopeVertex extends PolytopeVertexBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> implements Simplex
{
   public ExtendedPolytopeVertex()
   {
      super(new Point3D());
   }
   
   public ExtendedPolytopeVertex(double x, double y, double z)
   {
      super(new Point3D(x,y,z));
   }

   public ExtendedPolytopeVertex(Point3D position)
   {
      super(new Point3D(position));
   }

   public ExtendedPolytopeVertex(ExtendedPolytopeVertex vertex)
   {
      super(new Point3D());
      set(vertex);
   }
   
   @Override
   public PolytopeHalfEdge getAssociatedEdge(int index)
   {
      return (PolytopeHalfEdge) super.getAssociatedEdge(index);
   }

   public void set(ExtendedPolytopeVertex other)
   {
      
   }
}
