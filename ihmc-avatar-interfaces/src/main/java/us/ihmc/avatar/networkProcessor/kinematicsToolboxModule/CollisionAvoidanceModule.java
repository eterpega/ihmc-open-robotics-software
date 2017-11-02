package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import gnu.trove.map.hash.THashMap;
import us.ihmc.avatar.collisionAvoidance.FrameConvexPolytopeVisualizer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.CollisionAvoidanceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commons.PrintTools;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * Simple module that checks if the rigid bodies are colliding with the list of obstacles submitted
 * and generates collision avoidance commands for the optimization module
 * TODO include method for checking self collisions
 * @author Apoorv S
 */
public class CollisionAvoidanceModule
{
   /**
    * Enable collision box visualization 
    */
   private final static boolean visualizeRigidBodyMeshes = false;
   private final static boolean visualizeObstacleMeshes = true;
   private final static boolean visualizeCollisionVectors = true;
   private final static boolean debug = false;
   
   /**
    * Stores the collision detectors for the individual rigid bodies the idea here being that simplices can be 
    * carried forward into the next iteration to speed up the process
    */
   private THashMap<RigidBody, RigidBodyCollisionDetector> collisionDetectorMap = new THashMap<>();

   /**
    * Stores the provided obstacle / collision object meshes
    */
   private List<FrameConvexPolytope> obstacleMeshes = new ArrayList<>();

   /**
    * List of robot rigid bodies that the class will test for collisions
    */
   private List<RigidBody> rigidBodies;

   /**
    * List of rigid bodies that can collide with each other 
    * TODO this is unimplemented functionality
    */
   private List<Pair<RigidBody, RigidBody>> selfCollisionList = new ArrayList<>();

   /**
    * Settings for collision detection and avoidance
    */
   private final CollisionAvoidanceModuleSettings parameters = new CollisionAvoidanceModuleSettings();
   
   /**
    * Maintains the status if the collision avoidance is enabled
    */
   private boolean isEnabled = false;
   
   /**
    * List of collision avoidance commands to submit to the controller core
    */
   private final InverseKinematicsCommandList commandList = new InverseKinematicsCommandList();
   
   private final CollisionAvoidanceCommand command;
   
   /**
    * Generates the collision avoidance commands based on collision data from the collision detectors
    */
   private final CollisionAvoidanceCommandGenerator commandGenerator;
   
   /**
    * Visualization stuff
    */
   private final FrameConvexPolytopeVisualizer viz;
   
   public CollisionAvoidanceModule(RigidBody rootBody, InverseDynamicsJoint[] controlledJoints)
   {
      this(rootBody, controlledJoints, null);
   }
   
   public CollisionAvoidanceModule(RigidBody rootBody, InverseDynamicsJoint[] controlledJoints, FrameConvexPolytopeVisualizer viz)
   {
      command = new CollisionAvoidanceCommand(controlledJoints);
      commandGenerator = new CollisionAvoidanceCommandGenerator(rootBody, command, viz);
      this.viz = viz;
   }
   
   public void setRigidBodyCollisionMeshes(THashMap<RigidBody, FrameConvexPolytope> collisionMeshMap)
   {
      if(collisionMeshMap == null)
         return;
      this.rigidBodies = new ArrayList<>(collisionMeshMap.keySet());
      for(int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         collisionDetectorMap.put(rigidBody, new RigidBodyCollisionDetector(rigidBody, collisionMeshMap.get(rigidBody), parameters, commandGenerator));
         if(visualizeRigidBodyMeshes)
            visualize(collisionMeshMap.get(rigidBody));
      }
   }
   
   public void submitSelfCollisionPair(RigidBody rigidBody1, RigidBody rigidBody2)
   {
      selfCollisionList.add(new ImmutablePair<RigidBody, RigidBody>(rigidBody1, rigidBody2));
   }
   
   public void submitObstacleCollisionMesh(FrameConvexPolytope obstacleCollisionMesh)
   {
      this.obstacleMeshes.add(obstacleCollisionMesh);
      if(visualizeObstacleMeshes)
         visualize(obstacleCollisionMesh);
   }
   
   public void submitObstacleCollisionMesh(FrameConvexPolytope... obstacleMeshes)
   {
      for(int i = 0; i < obstacleMeshes.length; i++)
      {
         this.obstacleMeshes.add(obstacleMeshes[i]);
         if(visualizeObstacleMeshes)
            visualize(obstacleMeshes[i]);
      }
   }
   
   public void submitObstacleCollisionMesh(List<FrameConvexPolytope> obstacleMeshes)
   {
      this.obstacleMeshes.addAll(obstacleMeshes);
      if(visualizeObstacleMeshes)
      {
         for(int i = 0; i < obstacleMeshes.size(); i++)
            visualize(obstacleMeshes.get(i));
      }
   }
   
   public void clearObstacleMeshList()
   {
      if (viz != null)
      {
         for(int i = 0; i < obstacleMeshes.size(); i++)
            viz.removePolytope(obstacleMeshes.get(i));
      }
      obstacleMeshes.clear();
   }
   
   public boolean checkCollisionsAndAddAvoidanceCommands()
   {
      commandList.clear();
      if(!isEnabled)
         return false;
      if(debug)
         PrintTools.debug("Checking for collisions...");
      command.reset();
      boolean collisionDetected = false;
      if(viz != null && visualizeCollisionVectors)
         viz.clearCollisionVectors();
      for(int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         if(debug)
            PrintTools.debug("Checking collisions: " + rigidBody.toString());
         RigidBodyCollisionDetector collisionDetector = collisionDetectorMap.get(rigidBody);
         collisionDetected |= collisionDetector.checkCollisionsWithObstacles(obstacleMeshes);
      }
      if(viz != null && (visualizeRigidBodyMeshes || visualizeObstacleMeshes))
         viz.update();
      
      if(collisionDetected)
         commandList.addCommand(command);
      
      if(debug)
         PrintTools.debug("Done checking collisions");
      
      return collisionDetected;
   }

   public boolean isEnabled()
   {
      return isEnabled;
   }

   public void enable(boolean enabled)
   {
      this.isEnabled = enabled;
   }

   public InverseKinematicsCommandList getCollisionAvoidanceCommands()
   {
      return commandList;
   }
   
   private void visualize(FrameConvexPolytope polytope)
   {
      if(viz != null)
      {
         viz.addPolytope(polytope, Color.BLUE);
         viz.update();
      }
   }
}
