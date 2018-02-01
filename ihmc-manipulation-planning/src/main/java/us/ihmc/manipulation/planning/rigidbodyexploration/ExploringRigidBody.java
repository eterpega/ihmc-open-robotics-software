package us.ihmc.manipulation.planning.rigidbodyexploration;

import us.ihmc.euclid.geometry.Pose3D;

public class ExploringRigidBody
{
   private final Pose3D spatialData;
   private ExploringRigidBody parentRigidBody;

   public ExploringRigidBody()
   {
      this.spatialData = new Pose3D();
   }

   public ExploringRigidBody(Pose3D spatialData)
   {
      this.spatialData = new Pose3D(spatialData);
   }

   public void setSpatialData(Pose3D spatialData)
   {
      this.spatialData.set(spatialData);
   }

   public void setParentRigidBody(ExploringRigidBody parentRigidBody)
   {
      this.parentRigidBody = parentRigidBody;
   }

   public double getPositionDistance(Pose3D pose)
   {
      return spatialData.getPositionDistance(pose);
   }

   public double getOrientationDistance(Pose3D pose)
   {
      return spatialData.getOrientationDistance(pose);
   }

   public double getDistance(Pose3D pose)
   {
      return getPositionDistance(pose) + 0.5*getOrientationDistance(pose);
   }

   public double getPositionDistance(ExploringRigidBody other)
   {
      return getPositionDistance(other.getSpatialData());
   }

   public double getOrientationDistance(ExploringRigidBody other)
   {
      return getOrientationDistance(other.getSpatialData());
   }

   public double getDistance(ExploringRigidBody other)
   {
      return getDistance(other.getSpatialData());
   }

   public double getDistanceToParent()
   {
      return getDistance(this.parentRigidBody);
   }

   public Pose3D getSpatialData()
   {
      return spatialData;
   }

}
