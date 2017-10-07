package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytopeTest
{   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000)
   public void testConvexPolytopeWithPyramid()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      PolytopeVertex vertexOne = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = new PolytopeVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = new PolytopeVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = new PolytopeVertex(0.5, 0.5, 1.0);
      polytope.addVertex(vertexOne);
      polytope.addVertex(vertexTwo);
      polytope.addVertex(vertexThree);
      polytope.addVertex(vertexFour);
      PrintTools.debug(polytope.toString());
      assertTrue(polytope.getNumberOfFaces() == 4);
      assertTrue(polytope.getNumberOfEdges() == 6);
      assertTrue(polytope.getNumberOfVertices() == 4);

      ConvexPolytopeFace firstFace = polytope.getFace(0);
      ConvexPolytopeFace secondFace = polytope.getFace(1);
      ConvexPolytopeFace thirdFace = polytope.getFace(2);
      ConvexPolytopeFace fourthFace = polytope.getFace(3);
      assertTrue(firstFace.getNumberOfEdges() == 3);
      assertTrue(secondFace.getNumberOfEdges() == 3);
      assertTrue(thirdFace.getNumberOfEdges() == 3);
      assertTrue(fourthFace.getNumberOfEdges() == 3);
      for(int i = 0; i < 3; i++)
      {
         assertTrue(firstFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(secondFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(thirdFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(fourthFace.getEdge(i).getTwinHalfEdge() != null);
      }
      
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000)
   public void testConvexPolytopeWithSquarePyramid()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      PolytopeVertex vertexOne = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = new PolytopeVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = new PolytopeVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = new PolytopeVertex(0.0, 1.0, 0.0);
      PolytopeVertex vertexFive = new PolytopeVertex(0.5, 0.5, 1.0);
      polytope.addVertex(vertexOne);
      polytope.addVertex(vertexTwo);
      polytope.addVertex(vertexThree);
      polytope.addVertex(vertexFour);
      polytope.addVertex(vertexFive);
      PrintTools.debug(polytope.toString());
      assertTrue(polytope.getNumberOfFaces() == 5);
      assertTrue(polytope.getNumberOfEdges() == 8);
      assertTrue(polytope.getNumberOfVertices() == 5);

      ConvexPolytopeFace firstFace = polytope.getFace(0);
      ConvexPolytopeFace secondFace = polytope.getFace(1);
      ConvexPolytopeFace thirdFace = polytope.getFace(2);
      ConvexPolytopeFace fourthFace = polytope.getFace(3);
      assertTrue(firstFace.getNumberOfEdges() == 4);
      assertTrue(secondFace.getNumberOfEdges() == 3);
      assertTrue(thirdFace.getNumberOfEdges() == 3);
      assertTrue(fourthFace.getNumberOfEdges() == 3);
      for(int i = 0; i < 4; i++)
      {
         assertTrue(firstFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(secondFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(thirdFace.getEdge(i).getTwinHalfEdge() != null);
      }
      for(int i = 0; i < 3; i++)
      {
         assertTrue(fourthFace.getEdge(i).getTwinHalfEdge() != null);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 100000)
   public void testConvexPolytopeWithWierdShape()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      PolytopeVertex vertexOne = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = new PolytopeVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = new PolytopeVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = new PolytopeVertex(0.5, 0.5, 1.0);
      PolytopeVertex vertexFive = new PolytopeVertex(0.0, 1.0, 1.0);
      polytope.addVertex(vertexOne);
      polytope.addVertex(vertexTwo);
      polytope.addVertex(vertexThree);
      polytope.addVertex(vertexFour);
      polytope.addVertex(vertexFive);
      PrintTools.debug(polytope.toString());
      assertTrue(polytope.getNumberOfFaces() == 6);
      assertTrue(polytope.getNumberOfEdges() == 9);
      assertTrue(polytope.getNumberOfVertices() == 5);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConvexPolytopeWithAUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      PolytopeVertex vertexOne = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = new PolytopeVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = new PolytopeVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = new PolytopeVertex(0.0, 1.0, 0.0);
      PolytopeVertex vertexFive = new PolytopeVertex(0.0, 0.0, 1.0);
      PolytopeVertex vertexSix = new PolytopeVertex(1.0, 0.0, 1.0);
      PolytopeVertex vertexSeven = new PolytopeVertex(1.0, 1.0, 1.0);
      PolytopeVertex vertexEight = new PolytopeVertex(0.0, 1.0, 1.0);

      polytope.addVertex(vertexOne);
      polytope.addVertex(vertexTwo);
      polytope.addVertex(vertexThree);
      polytope.addVertex(vertexFour);
      polytope.addVertex(vertexFive);
      polytope.addVertex(vertexSix);
      polytope.addVertex(vertexSeven);
      polytope.addVertex(vertexEight);
      
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      assertEquals(8, polytope.getNumberOfVertices());
      assertEquals(12, polytope.getNumberOfEdges());

      assertEquals(3, vertexOne.getNumberOfAssociatedEdges());
      assertEquals(3, vertexTwo.getNumberOfAssociatedEdges());
      assertEquals(3, vertexThree.getNumberOfAssociatedEdges());
      assertEquals(3, vertexFour.getNumberOfAssociatedEdges());
      assertEquals(3, vertexFive.getNumberOfAssociatedEdges());
      assertEquals(3, vertexSix.getNumberOfAssociatedEdges());
      assertEquals(3, vertexSeven.getNumberOfAssociatedEdges());
      assertEquals(3, vertexEight.getNumberOfAssociatedEdges());

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(1.0, 2.0, 3.0);
      polytope.applyTransform(transform);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 2.0, 3.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 3.0, 3.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(2.0, 2.0, 4.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 2.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 1.0, 3.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-3.0, 2.0, 3.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(-2.0, 2.0, 4.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      // Apply in reverse order to get back to unit box at origin.
      transform.setRotationEulerAndZeroTranslation(0.0, 0.0, -Math.PI / 2.0);
      polytope.applyTransform(transform);
      transform.setTranslationAndIdentityRotation(-1.0, -2.0, -3.0);
      polytope.applyTransform(transform);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(0.0, 0.0, 0.0), vertexOne.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 1.0, 0.0), vertexThree.getPosition(), Epsilons.ONE_TEN_BILLIONTH);
      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(1.0, 0.0, 1.0), vertexSix.getPosition(), Epsilons.ONE_TEN_BILLIONTH);

      Vector3D supportDirection = new Vector3D(1.0, 1.0, 1.0);
      Point3D supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexSeven.getPosition());

      supportDirection = new Vector3D(-1.0, -1.0, -1.0);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexOne.getPosition());

      supportDirection = new Vector3D(100.0, 0.01, -0.01);
      supportingVertex = polytope.getSupportingVertex(supportDirection);
      assertTrue(supportingVertex == vertexThree.getPosition());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPolytopeConstructor()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(100.0, 100.0, 0.5);
      assertEquals(8, cubeOne.getNumberOfVertices());
      assertEquals(12, cubeOne.getNumberOfEdges());
      List<PolytopeHalfEdge> edges = cubeOne.getEdges();
      assertEquals(12, edges.size());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoundingBoxes()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      BoundingBox3D boundingBox = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      polytope.getBoundingBox(boundingBox);

      assertTrue(boundingBox.getMinX() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMinY() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMinZ() == Double.NEGATIVE_INFINITY);
      assertTrue(boundingBox.getMaxX() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getMaxY() == Double.POSITIVE_INFINITY);
      assertTrue(boundingBox.getMaxZ() == Double.POSITIVE_INFINITY);
      
      PolytopeVertex vertexOne = new PolytopeVertex(0.0, 0.0, 0.0);
      PolytopeVertex vertexTwo = new PolytopeVertex(1.0, 0.0, 0.0);
      PolytopeVertex vertexThree = new PolytopeVertex(1.0, 1.0, 0.0);
      PolytopeVertex vertexFour = new PolytopeVertex(0.0, 1.0, 0.0);

      PolytopeVertex vertexFive = new PolytopeVertex(new Point3D(0.0, 0.0, 1.0));
      PolytopeVertex vertexSix = new PolytopeVertex(new Point3D(1.0, 0.0, 1.0));
      PolytopeVertex vertexSeven = new PolytopeVertex(new Point3D(1.0, 1.0, 1.0));
      PolytopeVertex vertexEight = new PolytopeVertex(new Point3D(0.0, 1.0, 1.0));

      polytope.addVertex(vertexOne);
      polytope.addVertex(vertexTwo);
      polytope.addVertex(vertexThree);
      polytope.addVertex(vertexFour);
      polytope.addVertex(vertexFive);
      polytope.addVertex(vertexSix);
      polytope.addVertex(vertexSeven);
      polytope.addVertex(vertexEight);

      polytope.getBoundingBox(boundingBox);
      
      Point3D minimumPoint = new Point3D();
      boundingBox.getMinPoint(minimumPoint);
      
      Point3D maximumPoint = new Point3D();
      boundingBox.getMaxPoint(maximumPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 0.0), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 1.0, 1.0), maximumPoint, 1e-10);
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(10, 20, 30);
      polytope.applyTransform(transform);
      
      polytope.getBoundingBox(boundingBox);
      
      boundingBox.getMinPoint(minimumPoint);
      boundingBox.getMaxPoint(maximumPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(10.0, 20.0, 30.0), minimumPoint, 1e-10);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(11.0, 21.0, 31.0), maximumPoint, 1e-10);
      
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForPackage(ConvexPolytopeTest.class);
   }
}
