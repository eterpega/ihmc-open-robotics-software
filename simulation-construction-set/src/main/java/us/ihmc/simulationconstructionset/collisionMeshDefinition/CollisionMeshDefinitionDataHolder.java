package us.ihmc.simulationconstructionset.collisionMeshDefinition;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.robotDescription.RobotDescription;

public class CollisionMeshDefinitionDataHolder
{
   protected List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = new ArrayList<>();

   public CollisionMeshDefinitionDataHolder()
   {

   }

   public void addCollisionMeshOnRobotDescription(RobotDescription robotDescription)
   {
      for (int i = 0; i < collisionMeshDefinitionDataList.size(); i++)
      {
         collisionMeshDefinitionDataList.get(i).addCollisionMesh(robotDescription);
         collisionMeshDefinitionDataList.get(i).addLinkGraphics(robotDescription);
      }
   }
}
