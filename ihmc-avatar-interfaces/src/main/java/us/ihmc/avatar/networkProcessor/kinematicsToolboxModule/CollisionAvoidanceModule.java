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
    * Visualization object for debugging the collision avoidance
    */
   private final FrameConvexPolytopeVisualizer visualizer;
   
   /**
    * This is the current estimate of the collision quality that is calculated based on the 
    * the smallest Euclidean distance rigid bodies need to move to be in a non-colliding configuration.
    * In case the collision avoidance is disabled it is set to zero
    */
   private double collisionQuality = 0.0;
   
   /**
    * @param rootBody the root body is the first rigid body of the kinematic chain that will be used to avoid collisions
    * @param controlledJoints the list of joints that will be used for avoiding collisions
    */
   public CollisionAvoidanceModule(RigidBody rootBody, InverseDynamicsJoint[] controlledJoints)
   {
      this(rootBody, controlledJoints, null);
   }
   
   /**
    * @param rootBody the root body is the first rigid body of the kinematic chain that will be used to avoid collisions
    * @param controlledJoints the list of joints that will be used for avoiding collisions
    * @param visualizer a visualizer that helps debug the module
    */
   public CollisionAvoidanceModule(RigidBody rootBody, InverseDynamicsJoint[] controlledJoints, FrameConvexPolytopeVisualizer visualizer)
   {
      command = new CollisionAvoidanceCommand(controlledJoints);
      commandGenerator = new CollisionAvoidanceCommandGenerator(rootBody, command, visualizer);
      this.visualizer = visualizer;
   }
   
   /**
    * Sets the collision meshes for the various rigid bodies that will be used to detect and avoid collisions 
    * @param collisionMeshMap a map that relates the rigid bodies to their meshes 
    */
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
   
   /**
    * Stores a pair of rigid bodies that can physically collide given all the joint limits
    * @param rigidBody1 a rigid body that can collide with {@code rigidBody2}
    * @param rigidBody2 a rigid body that can collide with {@code rigidBody1}
    * Note: Self-collision is not yet implemented
    */
   public void submitSelfCollisionPair(RigidBody rigidBody1, RigidBody rigidBody2)
   {
      selfCollisionList.add(new ImmutablePair<RigidBody, RigidBody>(rigidBody1, rigidBody2));
   }
   
   /**
    * Submits a collision mesh to the module that will be used for detecting collisions
    * @param obstacleCollisionMesh this typically represent a object in the surroundings
    */
   public void submitObstacleCollisionMesh(FrameConvexPolytope obstacleCollisionMesh)
   {
      this.obstacleMeshes.add(obstacleCollisionMesh);
      if(visualizeObstacleMeshes)
         visualize(obstacleCollisionMesh);
   }
   
   /**
    * Submits a list of collision meshes to the module that will be used for detecting collisions with rigid bodies
    * @param obstacleMeshes these typically represent objects in the surroundings
    */
   public void submitObstacleCollisionMesh(FrameConvexPolytope... obstacleMeshes)
   {
      for(int i = 0; i < obstacleMeshes.length; i++)
      {
         this.obstacleMeshes.add(obstacleMeshes[i]);
         if(visualizeObstacleMeshes)
            visualize(obstacleMeshes[i]);
      }
   }
   
   /**
    * Submits a list of collision meshes to the module that will be used for detecting collisions with rigid bodies
    * @param obstacleMeshes these typically represent objects in the surroundings
    */
   public void submitObstacleCollisionMesh(List<FrameConvexPolytope> obstacleMeshes)
   {
      this.obstacleMeshes.addAll(obstacleMeshes);
      if(visualizeObstacleMeshes)
      {
         for(int i = 0; i < obstacleMeshes.size(); i++)
            visualize(obstacleMeshes.get(i));
      }
   }
   
   /**
    * Clears the list of obstacle meshes that the module has been using for collision detection
    */
   public void clearObstacleMeshList()
   {
      if (visualizer != null)
      {
         for(int i = 0; i < obstacleMeshes.size(); i++)
            visualizer.removePolytope(obstacleMeshes.get(i));
      }
      obstacleMeshes.clear();
   }
   
   /**
    * Checks if the rigid bodies are colliding with the submitted list of obstacles and generates the 
    * list of commands that can will move the inverse kinematics solutions to non-colliding configurations
    * @return
    */
   public boolean checkCollisionsAndAddAvoidanceCommands()
   {
      commandList.clear();
      if(!isEnabled)
         return false;
      if(debug)
         PrintTools.debug("Checking for collisions...");
      command.reset();
      boolean collisionDetected = false;
      if(visualizer != null && visualizeCollisionVectors)
         visualizer.clearCollisionVectors();
      collisionQuality = 0.0;
      for(int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         if(debug)
            PrintTools.debug("Checking collisions: " + rigidBody.toString());
         RigidBodyCollisionDetector collisionDetector = collisionDetectorMap.get(rigidBody);
         collisionDetected |= collisionDetector.checkCollisionsWithObstacles(obstacleMeshes);
         collisionQuality += collisionDetector.getCollisionQuality();
      }
      if(visualizer != null && (visualizeRigidBodyMeshes || visualizeObstacleMeshes))
         visualizer.update();
      
      if(collisionDetected)
         commandList.addCommand(command);
      
      if(debug)
         PrintTools.debug("Done checking collisions");
      
      return collisionDetected;
   }
   
   /**
    * Returns the collision quality parameter. This is the largest distance that any of the colliding bodies need to move by to 
    * move out of a collision 
    * @return
    */
   public double getCollisionQuality()
   {
      return collisionQuality;
   }

   /**
    * Returns the status whether the module is working or not
    * @return
    */
   public boolean isEnabled()
   {
      return isEnabled;
   }

   /**
    * Sets the state of the module
    * @param enabled
    */
   public void enable(boolean enabled)
   {
      this.isEnabled = enabled;
   }

   /**
    * Returns the list of commands that will help avoid collisions
    * @return
    */
   public InverseKinematicsCommandList getCollisionAvoidanceCommands()
   {
      return commandList;
   }

   /**
    * Internal function that submits meshes for visualization to the visualization listener
    * @param polytope
    */
   private void visualize(FrameConvexPolytope polytope)
   {
      if(visualizer != null)
      {
         visualizer.addPolytope(polytope, Color.BLUE);
         visualizer.update();
      }
   }
}
