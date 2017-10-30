package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import gnu.trove.map.hash.THashMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RigidBodyCollisionDetector
{
   private final THashMap<FrameConvexPolytope, ExtendedSimplexPolytope> collidingObstacleSimplices = new THashMap<>(); 
   private final Point3D rigidBodyCollidingPoint = new Point3D();
   private final Point3D obstacleMeshCollidingPoint = new Point3D();
   private final Vector3D collsionVector = new Vector3D();
   
   private final HybridGJKEPACollisionDetector collisionDetector;
   private final CollisionAvoidanceCommandGenerator commandGenerator;
   private final RigidBody rigidBody;
   
   
   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params, CollisionAvoidanceCommandGenerator commandGenerator)
   {
      this.collisionDetector = new HybridGJKEPACollisionDetector(params.getCollisionDetectionEpsilon());
      this.collisionDetector.setPolytopeA(rigidBodyCollisionMesh);
      this.commandGenerator = commandGenerator;
      this.rigidBody = rigidBody;
   }
   
   public boolean checkCollisionsWithObstacles(List<FrameConvexPolytope> obstacleMeshes)
   {
      boolean collisionDetected = false;
      for(int i = 0; i < obstacleMeshes.size(); i++)
      {
         FrameConvexPolytope obstacleMesh = obstacleMeshes.get(i);
         ExtendedSimplexPolytope pairSimplex = collidingObstacleSimplices.get(obstacleMesh);
         if(pairSimplex == null)
            pairSimplex = new ExtendedSimplexPolytope();
         collisionDetector.setSimplex(pairSimplex);
         collisionDetector.setPolytopeB(obstacleMesh);
         if(collisionDetector.checkCollision())
         {
            collisionDetected = true;
            //collidingObstacleSimplices.put(obstacleMesh, pairSimplex);
            collisionDetector.runEPAExpansion();
            collisionDetector.getCollisionPoints(rigidBodyCollidingPoint, obstacleMeshCollidingPoint);
            PrintTools.debug("Detected collision for:  " + rigidBody.getName());
            registerCollision(rigidBodyCollidingPoint, obstacleMeshCollidingPoint);
         }  
      }
      if(collisionDetected)
         PrintTools.debug("Got collision for: " + rigidBody.getName());
      return collisionDetected;
   }
   
   private void registerCollision(Point3D rigidBodyPoint, Point3D obstaclePoint)
   {
      // Get direction to move in to avoid collision
      collsionVector.sub(obstaclePoint, rigidBodyPoint);
      commandGenerator.addCollisionConstraint(rigidBody, rigidBodyPoint, collsionVector);
   }
}
