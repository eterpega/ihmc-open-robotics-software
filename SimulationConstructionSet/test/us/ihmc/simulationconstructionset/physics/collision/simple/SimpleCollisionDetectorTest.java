package us.ihmc.simulationconstructionset.physics.collision.simple;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.Contacts;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.gdx.GdxCollisionDetector;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

public class SimpleCollisionDetectorTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSphereToSphereCollisions()
   {
      SimpleCollisionDetector detector = new SimpleCollisionDetector();
//      GdxCollisionDetector detector = new GdxCollisionDetector(10.0);

      CollisionShapeFactory shapeFactory = detector.getShapeFactory();

      double radiusOne = 0.5;
      CollisionShape collideableObjectOne = createSphere(shapeFactory, radiusOne);
      CollisionShape collideableObjectTwo = createSphere(shapeFactory, radiusOne);

      double delta = 0.01;
      setCollisionObjectPosition(collideableObjectOne, 0.0, 0.0, 0.0);
      //TODO: 1.01 doesn't work with GDX solver. It reports a collision...
      setCollisionObjectPosition(collideableObjectTwo, 1.04 + delta, 0.0, 0.0);

      CollisionDetectionResult result = new CollisionDetectionResult();
      detector.performCollisionDetection(result);

      assertEquals(0, result.getNumberOfCollisions());

      setCollisionObjectPose(collideableObjectOne, 0.0, 0.0, 0.0, 0.3, 0.7, 0.9);
      setCollisionObjectPosition(collideableObjectTwo, 1.0 - delta, 0.0, 0.0);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      Contacts collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      CollisionShape shapeA = collision.getShapeA();
      CollisionShape shapeB = collision.getShapeB();

      Point3d locationA = new Point3d();
      Point3d locationB = new Point3d();
      Vector3d normal = new Vector3d();

      double distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA()) normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      JUnitTools.assertTuple3dEquals(new Vector3d(1.0, 0.0, 0.0), normal, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.5, 0.0, 0.0), locationA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.49, 0.0, 0.0), locationB, 1e-7);
      assertEquals(-delta, distance, 1e-7);

      // Another sphere to sphere test
      setCollisionObjectPosition(collideableObjectOne, -0.6, -0.1, 0.13);
      setCollisionObjectPosition(collideableObjectTwo, -0.4, 0.85, 0.3);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA()) normal.scale(-1.0);

      if (shapeA != collideableObjectOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == collideableObjectOne);
      assertTrue(shapeB == collideableObjectTwo);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.20292284665980928, 0.9638835216340943, 0.1724844196608379), normal, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(-0.4985385766700953, 0.3819417608170471, 0.21624220983041897), locationA, 1e-7); 
      JUnitTools.assertTuple3dEquals(new Vector3d(-0.5014614233299046, 0.36805823918295283, 0.21375779016958102), locationB, 1e-7);
      assertEquals(-0.014403733773306171, distance, 1e-7);
   }

  

   private CollisionShape createSphere(CollisionShapeFactory shapeFactory, double radius)
   {
      CollisionShapeDescription sphere = shapeFactory.createSphere(radius);
      CollisionShape collideableSphere = shapeFactory.addShape(sphere);
      return collideableSphere;
   }

   @DeployableTestMethod(estimatedDuration = 0.0, targets = TestPlanTarget.InDevelopment)
   @Test//(timeout = 30000)
   public void testBoxToBoxCollisions()
   {
      SimpleCollisionDetector detector = new SimpleCollisionDetector();
//      GdxCollisionDetector detector = new GdxCollisionDetector(10.0);

      CollisionShapeFactory shapeFactory = detector.getShapeFactory();

      double halfLengthX = 0.5;
      double halfWidthY = 0.6;
      double halfHeightZ = 0.7;

      CollisionShape boxOne = createBox(shapeFactory, halfLengthX, halfWidthY, halfHeightZ);
      CollisionShape boxTwo = createBox(shapeFactory, halfLengthX, halfWidthY, halfHeightZ);

      double delta = 0.01;
      setCollisionObjectPosition(boxOne, 0.0, 0.0, 0.0);
      setCollisionObjectPosition(boxTwo, 2.0*halfLengthX+delta, 2.0*halfWidthY+delta, 2.0*halfHeightZ+delta);

      CollisionDetectionResult result = new CollisionDetectionResult();
      detector.performCollisionDetection(result);

      assertEquals(0, result.getNumberOfCollisions());

      setCollisionObjectPosition(boxOne, 0.0, 0.0, 0.0);
      setCollisionObjectPosition(boxTwo, 2.0*halfLengthX-delta, 2.0*halfWidthY-delta, 2.0*halfHeightZ-delta);

      result.clear();
      detector.performCollisionDetection(result);

      assertEquals(1, result.getNumberOfCollisions());
      Contacts collision = result.getCollision(0);

      assertEquals(1, collision.getNumberOfContacts());

      CollisionShape shapeA = collision.getShapeA();
      CollisionShape shapeB = collision.getShapeB();

      Point3d locationA = new Point3d();
      Point3d locationB = new Point3d();
      Vector3d normal = new Vector3d();

      double distance = collision.getDistance(0);
      collision.getWorldA(0, locationA);
      collision.getWorldB(0, locationB);
      collision.getWorldNormal(0, normal);
      if (!collision.isNormalOnA()) normal.scale(-1.0);

      if (shapeA != boxOne)
      {
         CollisionShape tempShape = shapeA;
         shapeA = shapeB;
         shapeB = tempShape;

         Point3d tempLocation = locationA;
         locationA = locationB;
         locationB = tempLocation;

         normal.scale(-1.0);
      }

      assertTrue(shapeA == boxOne);
      assertTrue(shapeB == boxTwo);

      JUnitTools.assertTuple3dEquals(new Vector3d(-1.0, 0.0, 0.0), normal, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.49, 0.0, 0.0), locationA, 1e-7);
      JUnitTools.assertTuple3dEquals(new Vector3d(0.5, 0.0, 0.0), locationB, 1e-7);
      assertEquals(-delta, distance, 1e-7);
   }

   private void setCollisionObjectPosition(CollisionShape collisionOject, double x, double y, double z)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(x, y, z);
      collisionOject.setTransformToWorld(transform);      
   }
   
   private void setCollisionObjectPose(CollisionShape collisionObject, double x, double y, double z, double roll, double pitch, double yaw)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(roll, pitch, yaw);
      transform.setTranslation(x, y, z);
      collisionObject.setTransformToWorld(transform); 
   }

   private CollisionShape createBox(CollisionShapeFactory shapeFactory, double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      CollisionShapeDescription boxDescription = shapeFactory.createBox(halfLengthX, halfWidthY, halfHeightZ);
      return shapeFactory.addShape(boxDescription);
   }

}
