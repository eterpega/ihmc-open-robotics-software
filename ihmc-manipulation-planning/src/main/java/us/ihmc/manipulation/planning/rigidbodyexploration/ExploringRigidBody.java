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

   public double getPositionDistance(ExploringRigidBody other)
   {
      return spatialData.getPositionDistance(other.getSpatialData());
   }

   public double getOrientationDistance(ExploringRigidBody other)
   {
      return spatialData.getOrientationDistance(other.getSpatialData());
   }

   public double getDistance(ExploringRigidBody other)
   {
      return getPositionDistance(other) + getOrientationDistance(other);
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
