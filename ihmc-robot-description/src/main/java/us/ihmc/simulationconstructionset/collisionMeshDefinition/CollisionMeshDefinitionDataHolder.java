package us.ihmc.simulationconstructionset.collisionMeshDefinition;

import java.util.ArrayList;
import java.util.List;

public class CollisionMeshDefinitionDataHolder
{
   private List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = new ArrayList<>();

   public void addCollisionMeshDefinitionData(CollisionMeshDefinitionData collisionMeshData)
   {
      collisionMeshDefinitionDataList.add(collisionMeshData);
   }

   public List<CollisionMeshDefinitionData> getCollisionMeshDefinitionData()
   {
      return collisionMeshDefinitionDataList;
   }
}
