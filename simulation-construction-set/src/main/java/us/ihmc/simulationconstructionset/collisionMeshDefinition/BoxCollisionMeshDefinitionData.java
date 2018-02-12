package us.ihmc.simulationconstructionset.collisionMeshDefinition;

import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

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

   @Override
   public void addCollisionMesh(RobotDescription robotDescription)
   {
      LinkDescription linkDescription = robotDescription.getLinkDescription(parentJointName);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.translate(transformToParent.getTranslationX(), transformToParent.getTranslationY(), transformToParent.getTranslationZ());
      collisionMesh.rotate(transformToParent.getRotationMatrix());
      collisionMesh.addCubeReferencedAtCenter(length, width, height);
      collisionMesh.setCollisionGroup(collisionGroup);
      collisionMesh.setCollisionMask(collisionMask);
      linkDescription.addCollisionMesh(collisionMesh);
   }

   @Override
   public void addLinkGraphics(RobotDescription robotDescription)
   {
      LinkDescription linkDescription = robotDescription.getLinkDescription(parentJointName);

      LinkGraphicsDescription linkGraphics = linkDescription.getLinkGraphics();
      linkGraphics.identity();
      linkGraphics.transform(transformToParent);
      linkGraphics.addCube(length, width, height, yoAppearance);
   }
}
