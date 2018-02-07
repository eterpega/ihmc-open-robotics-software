package us.ihmc.simulationconstructionset.collisionMeshDefinition;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class CylinderCollisionMeshDefinitionData extends CollisionMeshDefinitionData
{
   private double radius = 1.0;
   private double height = 2.0;

   public CylinderCollisionMeshDefinitionData(String parentJointName)
   {
      super(parentJointName);
   }

   public CylinderCollisionMeshDefinitionData(String parentJointName, double radius, double height)
   {
      super(parentJointName);
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

   @Override
   public void addCollisionMesh(RobotDescription robotDescription)
   {
      LinkDescription linkDescription = robotDescription.getLinkDescription(parentJointName);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.translate(transformToParent.getTranslationX(), transformToParent.getTranslationY(), transformToParent.getTranslationZ());
      collisionMesh.rotate(new RotationMatrix(transformToParent.getRotationMatrix()));
      collisionMesh.addCylinderReferencedAtBottomMiddle(radius, height);
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
      linkGraphics.addCylinder(height, radius, yoAppearance);
   }

}
