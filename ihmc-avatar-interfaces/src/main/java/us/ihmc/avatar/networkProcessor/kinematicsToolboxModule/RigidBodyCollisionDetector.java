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
   //private final THashMap<FrameConvexPolytope, ExtendedSimplexPolytope> collidingObstacleSimplices = new THashMap<>(); 
   private final Point3D rigidBodyCollidingPoint = new Point3D();
   private final Point3D obstacleMeshCollidingPoint = new Point3D();
   private final Vector3D collsionVector = new Vector3D();

   private final HybridGJKEPACollisionDetector collisionDetector;
   private final CollisionAvoidanceCommandGenerator commandGenerator;
   private final RigidBody rigidBody;

   private final ExtendedSimplexPolytope tempSimplex;
   private final FrameConvexPolytopeVisualizer viz;
   private double collisionQuality;

   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params,
                                     CollisionAvoidanceCommandGenerator commandGenerator)
   {
      this(rigidBody, rigidBodyCollisionMesh, params, commandGenerator, null);
   }

   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params,
                                     CollisionAvoidanceCommandGenerator commandGenerator, FrameConvexPolytopeVisualizer viz)
   {
      this.collisionDetector = new HybridGJKEPACollisionDetector(null, params.getCollisionDetectionEpsilon());
      this.collisionDetector.setPolytopeA(rigidBodyCollisionMesh);
      this.commandGenerator = commandGenerator;
      this.rigidBody = rigidBody;
      this.tempSimplex = new ExtendedSimplexPolytope();
      this.viz = viz;
   }

   public boolean checkCollisionsWithObstacles(List<FrameConvexPolytope> obstacleMeshes)
   {
      boolean collisionDetected = false;
      collisionQuality = 0.0;
      for (int i = 0; i < obstacleMeshes.size(); i++)
      {
         FrameConvexPolytope obstacleMesh = obstacleMeshes.get(i);
         //ExtendedSimplexPolytope pairSimplex = collidingObstacleSimplices.get(obstacleMesh);
         //if(pairSimplex == null)
         //{
         //   pairSimplex = tempSimplex;
         //   pairSimplex.clear();
         //}
         //   pairSimplex = new ExtendedSimplexPolytope();
         tempSimplex.clear();
         collisionDetector.setSimplex(tempSimplex);
         collisionDetector.setPolytopeB(obstacleMesh);
         if (collisionDetector.checkCollision())
         {
            collisionDetected = true;
            //collidingObstacleSimplices.put(obstacleMesh, pairSimplex);
            collisionDetector.runEPAExpansion();
            collisionDetector.getCollisionPoints(rigidBodyCollidingPoint, obstacleMeshCollidingPoint);
            if (viz != null)
               viz.showCollisionVector(rigidBodyCollidingPoint, obstacleMeshCollidingPoint);
            registerCollision(rigidBodyCollidingPoint, obstacleMeshCollidingPoint);
         }
      }
      return collisionDetected;
   }
   
   public double getCollisionQuality()
   {
     return collisionQuality;
   }

   private void registerCollision(Point3D rigidBodyPoint, Point3D obstaclePoint)
   {
      // TODO rigid body points are right now always returned in world frame. Once that is fixed the points will have to be converted to the same frame
      collsionVector.sub(obstaclePoint, rigidBodyPoint);
      collisionQuality += collsionVector.length();
      commandGenerator.addCollisionConstraint(rigidBody, rigidBodyPoint, collsionVector);
   }
}
