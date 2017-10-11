package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Vector3D;

public class HybridGJKEPACollisionDetectorTest
{
   @Test(timeout = 1000)
   public void testSupportVectorDirectionNegative()
   {
      HybridGJKEPACollisionDetector collisionDetector = new HybridGJKEPACollisionDetector();
      collisionDetector.setSupportVectorDirection(new Vector3D(1, -1, 10));
      Vector3D vectorToGet = new Vector3D();
      collisionDetector.getSupportVectorDirectionNegative(vectorToGet);
      assertTrue(vectorToGet.getX() == -1);
      assertTrue(vectorToGet.getY() == 1);
      assertTrue(vectorToGet.getZ() == -10);
   }  
   
   @Test(timeout = 1000)
   public void testCollisionDetection()
   {
      ConvexPolytope polytope1 = ConvexPolytopeConstructor
   }
}
