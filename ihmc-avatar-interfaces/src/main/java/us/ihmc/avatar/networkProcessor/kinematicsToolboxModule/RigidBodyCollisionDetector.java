package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import gnu.trove.map.hash.THashMap;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
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
   
   private final ExtendedSimplexPolytope tempSimplex;
   
   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params, CollisionAvoidanceCommandGenerator commandGenerator)
   {
      this(rigidBody, rigidBodyCollisionMesh, params, commandGenerator, null);
   }
   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params, CollisionAvoidanceCommandGenerator commandGenerator, FrameConvexPolytopeVisualizer viz)
   {
      this.collisionDetector = new HybridGJKEPACollisionDetector(params.getCollisionDetectionEpsilon());
      this.collisionDetector.setPolytopeA(rigidBodyCollisionMesh);
      this.commandGenerator = commandGenerator;
      this.rigidBody = rigidBody;
      this.tempSimplex = new ExtendedSimplexPolytope();
      if(viz != null)
         viz.addPolytope(tempSimplex.getPolytope());
   }
   
   public boolean checkCollisionsWithObstacles(List<FrameConvexPolytope> obstacleMeshes)
   {
      boolean collisionDetected = false;
      for(int i = 0; i < obstacleMeshes.size(); i++)
      {
         FrameConvexPolytope obstacleMesh = obstacleMeshes.get(i);
         ExtendedSimplexPolytope pairSimplex = collidingObstacleSimplices.get(obstacleMesh);
         if(pairSimplex == null)
         {
            pairSimplex = tempSimplex;
            pairSimplex.clear();
         }
            //   pairSimplex = new ExtendedSimplexPolytope();
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
      return collisionDetected;
   }
   
   private void registerCollision(Point3D rigidBodyPoint, Point3D obstaclePoint)
   {
      // Get direction to move in to avoid collision
      collsionVector.sub(rigidBodyPoint, obstaclePoint);
      commandGenerator.addCollisionConstraint(rigidBody, rigidBodyPoint, collsionVector);
   }
}
