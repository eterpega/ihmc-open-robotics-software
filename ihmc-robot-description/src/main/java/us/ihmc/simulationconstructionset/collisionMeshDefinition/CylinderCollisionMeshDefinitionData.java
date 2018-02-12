package us.ihmc.simulationconstructionset.collisionMeshDefinition;

public class CylinderCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double radius = 1.0;
   private double height = 2.0;

   public CylinderCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
      setCollisionMeshType(CollisionMeshType.CYLINDER);
   }

   public CylinderCollisionMeshDefinitionData(String parentJointName, double radius, double height)
   {
      super(parentJointName);
      setCollisionMeshType(CollisionMeshType.CYLINDER);
      this.radius = radius;
      this.height = height;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }
}
