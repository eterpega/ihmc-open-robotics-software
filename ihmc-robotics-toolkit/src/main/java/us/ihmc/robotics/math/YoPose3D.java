package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

// TODO: implements Pose3DBasics, GeometryObject<YoPose3D>
public class YoPose3D
{
   private final YoPoint3D point;

   private final YoQuaternion orientation;

   public YoPose3D(String namePrefix, YoVariableRegistry registry)
   {
      point = new YoPoint3D(String.format("%s_point", namePrefix), registry);
      orientation = new YoQuaternion(String.format("%s_orientation", namePrefix), registry);
   }

   public YoPose3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      point = new YoPoint3D(String.format("%s_point", namePrefix), nameSuffix, registry);
      orientation = new YoQuaternion(String.format("%s_orientation", namePrefix), nameSuffix, registry);
   }

   //@Override
   public void setX(double x)
   {
      point.setX(x);
   }

   //@Override
   public void setY(double y)
   {
      point.setY(y);
   }

   //@Override
   public void setZ(double z)
   {
      point.setZ(z);
   }

   //@Override
   public double getX()
   {
      return point.getX();
   }

   //@Override
   public double getY()
   {
      return point.getY();
   }

   //@Override
   public double getZ()
   {
      return point.getZ();
   }

   //@Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   //@Override
   public double getPitch()
   {
      return orientation.getPitch();
   }

   //@Override
   public double getRoll()
   {
      return orientation.getRoll();
   }
}
