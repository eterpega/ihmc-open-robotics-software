package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.hash.THashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.CollisionAvoidanceCommand;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedSimplexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;

public class RigidBodyCollisionDetector
{
   private final THashMap<FrameConvexPolytope, ExtendedSimplexPolytope> collidingObstacleSimplices = new THashMap<>(); 
   private final HybridGJKEPACollisionDetector collisionDetector;
   private final List<Boolean> resultList = new ArrayList<>();

   private ExtendedSimplexPolytope tempSimplex = new ExtendedSimplexPolytope();
   
   public RigidBodyCollisionDetector(FrameConvexPolytope rigidBodyCollisionMesh, CollisionAvoidanceModuleSettings params)
   {
      this.collisionDetector = new HybridGJKEPACollisionDetector(params.getCollisionDetectionEpsilon());
      this.collisionDetector.setPolytopeA(rigidBodyCollisionMesh);
   }
   
   public List<Boolean> checkCollisionsWithObstacles(List<FrameConvexPolytope> obstacleMeshes)
   {
      resultList.clear();
      for(int i = 0; i < obstacleMeshes.size(); i++)
      {
         FrameConvexPolytope obstacleMesh = obstacleMeshes.get(i);
         ExtendedSimplexPolytope pairSimplex = collidingObstacleSimplices.get(obstacleMesh);
         if(pairSimplex == null)
            pairSimplex = new ExtendedSimplexPolytope();
         collisionDetector.setSimplex(pairSimplex);
         collisionDetector.setPolytopeB(obstacleMesh);
         resultList.add(collisionDetector.checkCollision());
         if(resultList.get(i))
         {
            collisionDetector.runEPAExpansion();
         }
         else
         {
            
         }
      }
      return resultList;
   }
}
