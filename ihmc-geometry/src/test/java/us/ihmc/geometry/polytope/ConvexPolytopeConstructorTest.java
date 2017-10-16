package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;

public class ConvexPolytopeConstructorTest
{
   @Test(timeout = 10000)
   public void testCylinderConstructor()
   {
      int numberOfSide = 20;
      ConvexPolytope cylinder = ConvexPolytopeConstructor.constructCylinder(new Point3D(), 10.0, 2.0, numberOfSide);
      assertTrue(cylinder != null);
      assertTrue(cylinder.getNumberOfFaces() == numberOfSide + 2);
      for (int j = 0; j < cylinder.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = cylinder.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }
   }
   
   @Test
   public void testSphereConstructor()
   {
      int recursionLevel = 0;
      ConvexPolytope sphere = ConvexPolytopeConstructor.constructSphere(1.0, new Point3D(), recursionLevel);
      assertTrue(sphere != null);
      for (int j = 0; j < sphere.getNumberOfFaces(); j++)
      {
         ConvexPolytopeFace face = sphere.getFace(j);
         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            assertTrue("Null twin edge for edge: " + face.getEdge(i).toString() + " on face: " + face.toString(), face.getEdge(i).getTwinHalfEdge() != null);
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getOriginVertex() == face.getEdge(i).getDestinationVertex());
            assertTrue("Twin edge: " + face.getEdge(i).getTwinHalfEdge().toString() + " mismatch for edge: " + face.getEdge(i).toString() + " on face: "
                  + face.toString(), face.getEdge(i).getTwinHalfEdge().getDestinationVertex() == face.getEdge(i).getOriginVertex());
         }
      }
   }
}
