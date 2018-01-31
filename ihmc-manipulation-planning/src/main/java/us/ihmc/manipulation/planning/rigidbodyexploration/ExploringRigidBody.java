package us.ihmc.manipulation.planning.rigidbodyexploration;

import us.ihmc.euclid.geometry.Pose3D;

public class ExploringRigidBody
{
   private final Pose3D spatialData;
   private ExploringRigidBody parentRigidBody;

   public ExploringRigidBody(Pose3D pose)
   {
      this.spatialData = new Pose3D(pose);
   }

   public double getPositionDistance(ExploringRigidBody other)
   {

      return 0.0;
   }

   public double getOrientationDistance(ExploringRigidBody other)
   {

      return 0.0;
   }

   public double getDistance(ExploringRigidBody other)
   {

      return 0.0;
   }

   public double getDistanceToParent()
   {

      return 0.0;
   }

   public Pose3D getSpatialData()
   {
      return spatialData;
   }

}
