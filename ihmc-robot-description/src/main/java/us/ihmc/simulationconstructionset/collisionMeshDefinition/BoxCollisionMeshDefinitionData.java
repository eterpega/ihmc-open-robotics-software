package us.ihmc.simulationconstructionset.collisionMeshDefinition;

public class BoxCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double length = 1.0;
   private double width = 1.0;
   private double height = 1.0;

   public BoxCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
   }

   public BoxCollisionMeshDefinitionData(String parentJointName, double length, double width, double height)
   {
      super(parentJointName);
      this.length = length;
      this.width = width;
      this.height = height;
   }

   public void setLength(double length)
   {
      this.length = length;
   }

   public void setWidth(double width)
   {
      this.width = width;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public double getLength()
   {
      return length;
   }

   public double getWidth()
   {
      return width;
   }

   public double getHeight()
   {
      return height;
   }
}
