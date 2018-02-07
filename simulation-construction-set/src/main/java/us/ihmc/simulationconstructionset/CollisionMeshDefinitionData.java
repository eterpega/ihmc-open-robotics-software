package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.RobotDescription;

public abstract class CollisionMeshDefinitionData
{
   //   enum CollisionMeshType
   //   {
   //      Sphere, Box, Cylinder, Capsule
   //   }
   //
   //   private CollisionMeshType collisionMeshType;

   protected String parentJointName;

   /**
    * CollisionMesh will be created after transform from the parent joint.
    */
   protected RigidBodyTransform transformToParent = new RigidBodyTransform();

   protected AppearanceDefinition yoAppearance = YoAppearance.Beige();

   protected int collisionGroup = 0xffffffff;
   protected int collisionMask = 0xffffffff;

   public CollisionMeshDefinitionData(String parentJointName)
   {
      this.parentJointName = parentJointName;
   }

   public void setTransformToParent(RigidBodyTransform transformToParent)
   {
      this.transformToParent.set(transformToParent);
   }

   public void setYoApperance(AppearanceDefinition yoAppearance)
   {
      this.yoAppearance = yoAppearance;
   }

   public abstract void addCollisionMesh(RobotDescription robotDescription);

   public abstract void addLinkGraphics(RobotDescription robotDescription);
}
