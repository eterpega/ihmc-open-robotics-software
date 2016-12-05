package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class SimplePlanarRegionBipedalAnytimeFootstepPlanner extends PlanarRegionBipedalFootstepPlanner implements AnytimeFootstepPlanner, Runnable
{
   private BipedalFootstepPlannerNode closestNodeToGoal = null;
   private final AtomicReference<FootstepPlan> bestPlanYet = new AtomicReference<>(null);
   private boolean stopRequested = false;
   private boolean isBestPlanYetOptimal = false;
   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>(null);
   private final AtomicReference<BipedalFootstepPlannerNode> newStartNodeReference = new AtomicReference<>(null);
   private boolean requestInitialize = false;
   private final IntegerYoVariable maxNumberOfNodesBeforeSleeping = new IntegerYoVariable("maxNumberOfNodesBeforeSleeping", registry);

   public SimplePlanarRegionBipedalAnytimeFootstepPlanner(YoVariableRegistry parentRegistry)
   {
      super(parentRegistry);
      maxNumberOfNodesBeforeSleeping.set(5000);
   }

   /**
    * @return The FootstepPlan that ends the closest to the goal of all explored yet
    */
   @Override
   public FootstepPlan getBestPlanYet()
   {
      return bestPlanYet.getAndSet(null);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionsListReference.set(planarRegionsList);
   }

   @Override
   protected void initialize()
   {
      stack.clear();
      startNode = new BipedalFootstepPlannerNode(initialSide, initialFootPose);
      stack.push(startNode);
      closestNodeToGoal = null;
      mapToAllExploredNodes.clear();
      bestPlanYet.set(null);
   }

   @Override
   public void executingFootstep(SimpleFootstep footstep)
   {
      FramePose solePose = new FramePose();
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      footstep.getSoleFramePose(solePose);

      solePose.getRigidBodyTransform(soleTransform);
      BipedalFootstepPlannerNode newStartNode = new BipedalFootstepPlannerNode(footstep.getRobotSide(), soleTransform);

      newStartNodeReference.set(newStartNode);
   }

   private void replaceStartNode()
   {
      BipedalFootstepPlannerNode newStartNode = newStartNodeReference.getAndSet(null);
      if(newStartNode != null)
      {
         ArrayList<BipedalFootstepPlannerNode> childrenOfStartNode = new ArrayList<>();
         startNode.getChildren(childrenOfStartNode);

         for(BipedalFootstepPlannerNode node : childrenOfStartNode)
         {
            if(!node.equals(newStartNode))
            {
               recursivelyMarkAsDead(node);
            }
         }

         startNode = newStartNode;
         initialSide = startNode.getRobotSide();
         initialFootPose = startNode.getTransformToParent();
      }
   }

   private void recursivelyMarkAsDead(BipedalFootstepPlannerNode node)
   {
      node.setToDead();
      ArrayList<BipedalFootstepPlannerNode> childrenNodes = new ArrayList<>();
      node.getChildren(childrenNodes);

      for(BipedalFootstepPlannerNode childNode : childrenNodes)
      {
         recursivelyMarkAsDead(childNode);
      }
   }

   @Override
   public boolean isBestPlanYetOptimal()
   {
      return isBestPlanYetOptimal;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      return plan(true);
   }

   public FootstepPlanningResult plan(boolean stopAndReturnWhenGoalIsFound)
   {
      initialize();
      goalNode = null;
      footstepPlan = null;

      planarRegionPotentialNextStepCalculator.setStartNode(startNode);

      while (!stopRequested)
      {
         replaceStartNode();

         if(requestInitialize)
            initialize();

         checkForNewPlanarRegionsList();

         if(stack.size() > maxNumberOfNodesBeforeSleeping.getIntegerValue())
            ThreadTools.sleep(100);

         if (stack.isEmpty())
         {
            if (stopAndReturnWhenGoalIsFound)
            {
               notifyListenerSolutionWasNotFound();
               return FootstepPlanningResult.NO_PATH_EXISTS;
            }

            ThreadTools.sleep(100);
            continue;
         }

         BipedalFootstepPlannerNode nodeToExpand = stack.pop();

         if(nodeToExpand.isDead())
            continue;

         boolean nearbyNodeAlreadyExists = checkIfNearbyNodeAlreadyExistsAndStoreIfNot(nodeToExpand);
         if (nearbyNodeAlreadyExists)
            continue;

         setNodesCostsAndRememberIfClosestYet(nodeToExpand);
         notifyListenerNodeForExpansionWasAccepted(nodeToExpand);

         if (nodeToExpand.isAtGoal())
         {
            if ((nodeToExpand.getParentNode() != null) && (nodeToExpand.getParentNode().isAtGoal()) && stopAndReturnWhenGoalIsFound)
            {
               goalNode = nodeToExpand;

               notifyListenerSolutionWasFound(getPlan());
               return FootstepPlanningResult.OPTIMAL_SOLUTION;
            }
         }

         expandChildrenAndAddNodes(stack, nodeToExpand);
      }

      notifyListenerSolutionWasNotFound();
      return FootstepPlanningResult.NO_PATH_EXISTS;
   }

   private void checkForNewPlanarRegionsList()
   {
      PlanarRegionsList planarRegionsList = planarRegionsListReference.getAndSet(null);
      if (planarRegionsList != null)
      {
         super.setPlanarRegions(planarRegionsList);
         initialize();
      }
   }

   private void setNodesCostsAndRememberIfClosestYet(BipedalFootstepPlannerNode nodeToSetCostOf)
   {
      Point3d currentPosition = nodeToSetCostOf.getSolePosition();
      Point3d goalPosition = planarRegionPotentialNextStepCalculator.getGoalPosition(nodeToSetCostOf.getRobotSide());
      Vector3d currentToGoalVector = new Vector3d();
      currentToGoalVector.sub(goalPosition, currentPosition);

      double costFromParent = 1.0;
      double costToHereFromStart;
      if (nodeToSetCostOf.getParentNode() == null)
      {
         costToHereFromStart = 0.0;
      }
      else
      {
         costToHereFromStart = nodeToSetCostOf.getParentNode().getCostToHereFromStart() + costFromParent;
      }
      nodeToSetCostOf.setCostFromParent(costFromParent);
      nodeToSetCostOf.setCostToHereFromStart(costToHereFromStart);

      double euclideanDistanceToGoal = currentToGoalVector.length();
      nodeToSetCostOf.setEstimatedCostToGoal(euclideanDistanceToGoal);

      if (closestNodeToGoal == null || euclideanDistanceToGoal < closestNodeToGoal.getEstimatedCostToGoal())
      {
         closestNodeToGoal = nodeToSetCostOf;
         FootstepPlan newBestPlan = FootstepPlanningUtils.createFootstepPlanFromEndNode(closestNodeToGoal);
         bestPlanYet.set(newBestPlan);
      }
   }

   public void requestStop()
   {
      stopRequested = true;
   }

   @Override
   public void run()
   {
      stopRequested = false;
      plan(false);
   }
}
