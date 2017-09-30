package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;

public class PolytopeVertexTest
{
   @Test
   public void testConstructors()
   {
         PolytopeVertex vertex1 = new PolytopeVertex(0.1, 0.2, 0.4);
         assertTrue(vertex1.getX() == 0.1);
         assertTrue(vertex1.getY() == 0.2);
         assertTrue(vertex1.getZ() == 0.4);
         
         PolytopeVertex vertex2 = new PolytopeVertex(new Point3D(1.2,  3.14, 1.519));
         assertTrue(vertex2.getX() == 1.2);
         assertTrue(vertex2.getY() == 3.14);
         assertTrue(vertex2.getZ() == 1.519);
         
         PolytopeVertex vertex3 = new PolytopeVertex(vertex1);
         assertTrue(vertex1.getX() == vertex3.getX());
         assertTrue(vertex1.getY() == vertex3.getY());
         assertTrue(vertex1.getZ() == vertex3.getZ());
   }
   
   @Test
   public void testEdgeAssociation()
   {
      PolytopeVertex vertex1 = new PolytopeVertex(0.1, 0.2, 0.3);
      PolytopeVertex vertex2 = new PolytopeVertex(0.2, 3.1, 5.7);
      PolytopeVertex vertex3 = new PolytopeVertex(1.2, 8.1, 0.0);
      PolytopeHalfEdge edge1 = new PolytopeHalfEdge(vertex1, vertex2);
      PolytopeHalfEdge edge2 = new PolytopeHalfEdge(vertex1, vertex3);
      assertTrue(vertex1.getAssociatedEdges().size() == 2);
      assertTrue(vertex1.getAssociatedEdges().get(0) == edge1);
      assertTrue(vertex1.getAssociatedEdges().get(1) == edge2);
      assertTrue(vertex2.getAssociatedEdges().size() == 0);
      assertTrue(vertex3.getAssociatedEdges().size() == 0);
      
      PolytopeHalfEdge twinEdge1 = edge1.createTwinHalfEdge();
      assertTrue(vertex2.getAssociatedEdges().size() == 1);
      assertTrue(vertex2.getAssociatedEdges().get(0) == twinEdge1);
   }
}
