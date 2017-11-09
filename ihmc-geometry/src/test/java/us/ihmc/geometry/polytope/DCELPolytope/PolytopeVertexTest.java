package us.ihmc.geometry.polytope.DCELPolytope;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;

public class PolytopeVertexTest
{
   @Test(timeout = 1000)
   public void testConstructors()
   {
      ExtendedPolytopeVertex vertex1 = new ExtendedPolytopeVertex(0.1, 0.2, 0.4);
      assertTrue(vertex1.getX() == 0.1);
      assertTrue(vertex1.getY() == 0.2);
      assertTrue(vertex1.getZ() == 0.4);

      ExtendedPolytopeVertex vertex2 = new ExtendedPolytopeVertex(new Point3D(1.2, 3.14, 1.519));
      assertTrue(vertex2.getX() == 1.2);
      assertTrue(vertex2.getY() == 3.14);
      assertTrue(vertex2.getZ() == 1.519);

      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(vertex1);
      assertTrue(vertex1.getX() == vertex3.getX());
      assertTrue(vertex1.getY() == vertex3.getY());
      assertTrue(vertex1.getZ() == vertex3.getZ());
   }

   @Test(timeout = 1000)
   public void testEdgeAssociation()
   {
      ExtendedPolytopeVertex vertex1 = new ExtendedPolytopeVertex(0.1, 0.2, 0.3);
      ExtendedPolytopeVertex vertex2 = new ExtendedPolytopeVertex(0.2, 3.1, 5.7);
      ExtendedPolytopeVertex vertex3 = new ExtendedPolytopeVertex(1.2, 8.1, 0.0);
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

   @Test(timeout = 1000)
   public void testFrameConstructor()
   {
      ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex();
      FramePolytopeVertex framevertex = new FramePolytopeVertex(ReferenceFrame.getWorldFrame(), new ExtendedPolytopeVertex());
      FramePolytopeVertex framevertex2 = new FramePolytopeVertex(ReferenceFrame.getWorldFrame(), new ExtendedPolytopeVertex());
      framevertex.epsilonEquals(framevertex2, Epsilons.ONE);
      vertex.epsilonEquals(framevertex, Epsilons.ONE);
   }

   @Test(timeout = 1000)
   public void testDotProduct()
   {
      ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex(1.1241252, -15.12415, 86.12536);
      assertTrue(vertex.dot(new Vector3D(1.0, 0.5, 1.0)) == 1.1241252 - 0.5*15.12415 + 86.12536);
   }
   
   @Test(timeout = 1000)
   public void testShortestDistanceCalculation()
   {
      ExtendedPolytopeVertex vertex = new ExtendedPolytopeVertex(1.1241252, -15.12415, 86.12536);
   }

}
