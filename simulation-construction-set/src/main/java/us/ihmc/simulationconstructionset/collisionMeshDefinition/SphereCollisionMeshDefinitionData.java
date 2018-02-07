package us.ihmc.simulationconstructionset.collisionMeshDefinition;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

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

   @Override
   public void addCollisionMesh(RobotDescription robotDescription)
   {
      LinkDescription linkDescription = robotDescription.getLinkDescription(parentJointName);

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.translate(transformToParent.getTranslationX(), transformToParent.getTranslationY(), transformToParent.getTranslationZ());
      collisionMesh.rotate(new RotationMatrix(transformToParent.getRotationMatrix()));
      collisionMesh.addSphere(radius);
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
      linkGraphics.addSphere(radius, yoAppearance);
   }

}
