package us.ihmc.manipulation.planning.rigidbodyexploration;

import java.util.Random;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SpatialDataRegion
{
   private final Random randomManager = new Random(1);
   /**
    * All limit boundary is defined from WorldFrame.
    */
   private final Vector3D positionLowerLimitSpace;
   private final Vector3D positionUpperLimitSpace;

   private final Vector3D rotationVectorLowerLimitSpace;
   private final Vector3D rotationVectorUpperLimitSpace;

   public SpatialDataRegion()
   {
      positionLowerLimitSpace = new Vector3D();
      positionUpperLimitSpace = new Vector3D();

      rotationVectorLowerLimitSpace = new Vector3D();
      rotationVectorUpperLimitSpace = new Vector3D();
   }

   public SpatialDataRegion(SpatialDataRegion other)
   {
      this();
      setPositionLowerLimitSpace(other.positionLowerLimitSpace);
      setPositionUpperLimitSpace(other.positionUpperLimitSpace);
      setRotationVectorLowerLimitSpace(other.rotationVectorLowerLimitSpace);
      setRotationVectorUpperLimitSpace(other.rotationVectorUpperLimitSpace);
   }

   public void setPositionLowerLimitSpace(Vector3D limitSpace)
   {
      positionLowerLimitSpace.set(limitSpace);
   }

   public void setPositionUpperLimitSpace(Vector3D limitSpace)
   {
      positionUpperLimitSpace.set(limitSpace);
   }

   public void setRotationVectorLowerLimitSpace(Vector3D limitSpace)
   {
      rotationVectorLowerLimitSpace.set(limitSpace);
   }

   public void setRotationVectorUpperLimitSpace(Vector3D limitSpace)
   {
      rotationVectorUpperLimitSpace.set(limitSpace);
   }

   public void setPositionLowerLimitSpace(double x, double y, double z)
   {
      positionLowerLimitSpace.set(x, y, z);
   }

   public void setPositionUpperLimitSpace(double x, double y, double z)
   {
      positionUpperLimitSpace.set(x, y, z);
   }

   public void setRotationVectorLowerLimitSpace(double x, double y, double z)
   {
      rotationVectorLowerLimitSpace.set(x, y, z);
   }

   public void setRotationVectorUpperLimitSpace(double x, double y, double z)
   {
      rotationVectorUpperLimitSpace.set(x, y, z);
   }

   private Pose3D getRandomSpatialData()
   {
      Pose3D spatialData = new Pose3D();

      spatialData.getPosition()
                 .setX(positionLowerLimitSpace.getX() + (positionUpperLimitSpace.getX() - positionLowerLimitSpace.getX()) * randomManager.nextDouble());
      spatialData.getPosition()
                 .setY(positionLowerLimitSpace.getY() + (positionUpperLimitSpace.getY() - positionLowerLimitSpace.getY()) * randomManager.nextDouble());
      spatialData.getPosition()
                 .setZ(positionLowerLimitSpace.getZ() + (positionUpperLimitSpace.getZ() - positionLowerLimitSpace.getZ()) * randomManager.nextDouble());

      Vector3D rotationVector = new Vector3D();

      rotationVector.setX(rotationVectorLowerLimitSpace.getX()
            + (rotationVectorLowerLimitSpace.getX() - rotationVectorUpperLimitSpace.getX()) * randomManager.nextDouble());
      rotationVector.setY(rotationVectorLowerLimitSpace.getY()
            + (rotationVectorLowerLimitSpace.getY() - rotationVectorUpperLimitSpace.getY()) * randomManager.nextDouble());
      rotationVector.setZ(rotationVectorLowerLimitSpace.getZ()
            + (rotationVectorLowerLimitSpace.getZ() - rotationVectorUpperLimitSpace.getZ()) * randomManager.nextDouble());

      spatialData.setOrientation(new Quaternion(rotationVector));

      return spatialData;
   }

   public void getRandomSpatialData(ExploringRigidBody exploringRigidBody)
   {
      exploringRigidBody.setSpatialData(getRandomSpatialData());
   }

   public void getRandomSpatialData(Pose3D pose)
   {
      pose.set(getRandomSpatialData());
   }

}
