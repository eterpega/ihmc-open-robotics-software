package us.ihmc.manipulation.planning.rigidbodyexploration;

import us.ihmc.euclid.tuple3D.Vector3D;

public class SpatialDataRegion
{
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
}
