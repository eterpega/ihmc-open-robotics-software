package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

import us.ihmc.commons.Epsilons;

public class PolytopeFaceTest
{
   @Test
   public void testConstructorAndAddVertex()
   {
      PolytopeFace face = new PolytopeFace();
      PolytopeVertex vertex1 = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertex2 = new PolytopeVertex(0.0, 1.0, 0.0);
      PolytopeVertex vertex3 = new PolytopeVertex(1.0, 0.0, 0.0);
      face.addVertex(vertex1);
      assertTrue(face.getNumberOfEdges() == 1);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex1);
      face.addVertex(vertex2);
      assertTrue(face.getNumberOfEdges() == 2);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex1);
      face.addVertex(vertex3);
      assertTrue(face.getNumberOfEdges() == 3);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex1);
      
      PolytopeVertex vertex4 = new PolytopeVertex(1.0, 1.0, 0.0);
      face.addVertex(vertex4);
      assertTrue(face.getNumberOfEdges() == 4);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex4);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex4);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex1);

      PolytopeVertex vertex5 = new PolytopeVertex(2.0, 2.0, 0.0);
      face.addVertex(vertex5);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 4, face.getNumberOfEdges() == 4);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex5);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex5);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex1);

      PolytopeVertex vertex6 = new PolytopeVertex(3.0, 3.0, 0.0);
      face.addVertex(vertex6);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 4, face.getNumberOfEdges() == 4);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex6);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex6);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex1);

      PolytopeVertex vertex7 = new PolytopeVertex(2.0, 3.0, 0.0);
      face.addVertex(vertex7);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex7);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex7);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex6);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex6);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(4).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(4).getDestinationVertex() == vertex1);

      PolytopeVertex vertex8 = new PolytopeVertex(0.0, 0.0, 1.0);
      assertFalse(face.isPointInFacePlane(vertex8, Epsilons.ONE_BILLIONTH));
      face.addVertex(vertex8);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex7);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex7);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex6);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex6);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(4).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(4).getDestinationVertex() == vertex1);
      
      PolytopeVertex vertex9 = new PolytopeVertex(1.0, 1.0, 0.0);
      assertTrue(face.isPointInFacePlane(vertex9, Epsilons.ONE_MILLIONTH));
      assertTrue(face.isInteriorPoint(vertex9));
      face.addVertex(vertex9);
      assertTrue("Number of edges: " + face.getNumberOfEdges() + ", needed: " + 5, face.getNumberOfEdges() == 5);
      assertTrue(face.getEdge(0).getOriginVertex() == vertex1);
      assertTrue(face.getEdge(0).getDestinationVertex() == vertex2);
      assertTrue(face.getEdge(1).getOriginVertex() == vertex2);
      assertTrue(face.getEdge(1).getDestinationVertex() == vertex7);
      assertTrue(face.getEdge(2).getOriginVertex() == vertex7);
      assertTrue(face.getEdge(2).getDestinationVertex() == vertex6);
      assertTrue(face.getEdge(3).getOriginVertex() == vertex6);
      assertTrue(face.getEdge(3).getDestinationVertex() == vertex3);
      assertTrue(face.getEdge(4).getOriginVertex() == vertex3);
      assertTrue(face.getEdge(4).getDestinationVertex() == vertex1);
   }
}
