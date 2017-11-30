package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.List;

import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class RigidBodyCollisionDetector
{
   //private final THashMap<FrameConvexPolytope, ExtendedSimplexPolytope> collidingObstacleSimplices = new THashMap<>(); 
   private final Point3D rigidBodyCollidingPoint = new Point3D();
   private final Point3D obstacleMeshCollidingPoint = new Point3D();
   private final Vector3D collisionVector = new Vector3D();

   private final HybridGJKEPACollisionDetector collisionDetector;

   private final ExtendedSimplexPolytope tempSimplex;
   private final FrameConvexPolytopeVisualizer viz;
   private double collisionQuality;

   private final RigidBodyTransform collisionFrameTransform = new RigidBodyTransform();
   private final ReferenceFrame collisionReferenceFrame;
   /**
    * The secret sauce that ensures that the resultant joint velocity is in a direction that avoids
    * collisions. This should be as high as possible but not so high that the solution becomes in
    * feasible
    */
   private double collisionAvoidanceTaskObjective = 500; //m/s 
   private final SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();

   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params)
   {
      this(rigidBody, rigidBodyCollisionMesh, params, null);
   }

   public RigidBodyCollisionDetector(RigidBody rigidBody, FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params,
                                     FrameConvexPolytopeVisualizer viz)
   {
      this.collisionDetector = new HybridGJKEPACollisionDetector(null, params.getCollisionDetectionEpsilon());
      this.collisionDetector.setPolytopeA(rigidBodyCollisionMesh);
      this.tempSimplex = new ExtendedSimplexPolytope();
      this.viz = viz;

      collisionReferenceFrame = new ReferenceFrame(rigidBody.getName() + "CollisionFrame", rigidBody.getBodyFixedFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(collisionFrameTransform);
         }
      };

      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.clearSelection();
      selectionMatrix.setSelectionFrames(null, collisionReferenceFrame);
      selectionMatrix.setLinearAxisSelection(false, false, true);
      spatialVelocityCommand.setSelectionMatrix(selectionMatrix);
      spatialVelocityCommand.set(ScrewTools.getRootBody(rigidBody), rigidBody);
      spatialVelocityCommand.setWeight(1.0);
   }

   public boolean checkCollisionsWithObstacles(List<FrameConvexPolytope> obstacleMeshes)
   {
      boolean collisionDetected = false;
      collisionQuality = 0.0;
      for (int i = 0; i < obstacleMeshes.size(); i++)
      {
         FrameConvexPolytope obstacleMesh = obstacleMeshes.get(i);
         tempSimplex.clear();
         collisionDetector.setSimplex(tempSimplex);
         collisionDetector.setPolytopeB(obstacleMesh);
         if (collisionDetector.checkCollision())
         {
            collisionDetected = true;
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
      collisionVector.sub(obstaclePoint, rigidBodyPoint);

      if (viz != null)
         viz.showCollisionVector(rigidBodyPoint, obstaclePoint);

      double penetrationDistance = collisionVector.length();
      collisionQuality += penetrationDistance;
      updateCollisionFrame(rigidBodyPoint, collisionVector);

      FrameVector3D desiredLinearVelocity = new FrameVector3D(collisionReferenceFrame);
      desiredLinearVelocity.setZ(collisionAvoidanceTaskObjective * penetrationDistance);
      spatialVelocityCommand.setLinearVelocity(collisionReferenceFrame, desiredLinearVelocity);
   }

   private void updateCollisionFrame(Point3D collisionPoint, Vector3D collisionVector)
   {
      collisionFrameTransform.setTranslation(collisionPoint);
      collisionFrameTransform.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(collisionVector));
      collisionFrameTransform.invertRotation();
      collisionReferenceFrame.update();
   }

   public InverseKinematicsCommand<?> getCommand()
   {
      return spatialVelocityCommand;
   }
}
