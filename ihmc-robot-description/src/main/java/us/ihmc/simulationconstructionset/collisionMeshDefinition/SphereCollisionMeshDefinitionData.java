package us.ihmc.simulationconstructionset.collisionMeshDefinition;

public class SphereCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double radius = 1.0;

   public SphereCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
   }

   public SphereCollisionMeshDefinitionData(String parentJointName, double radius)
   {
      super(parentJointName);
      this.radius = radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getRadius()
   {
      return radius;
   }
}
