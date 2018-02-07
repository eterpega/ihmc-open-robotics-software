package us.ihmc.atlas.parameters;

import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionDataHolder;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.SphereCollisionMeshDefinitionData;

public class AtlasCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public static double footLength = 0.28;
   public static double footWidth = 0.28;
   public static double footHeight = 0.28;
   
   public AtlasCollisionMeshDefinitionDataHolder()
   {
      CollisionMeshDefinitionData collisionMeshData = new SphereCollisionMeshDefinitionData("j_MainBody_Link_7L", 0.1);
      
      addCollisionMeshDefinitionData(collisionMeshData);
   }
}
