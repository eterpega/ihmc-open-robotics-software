package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

// TODO: implements Pose2DBasics, GeometryObject<YoPose2D>
public class YoPose2D
{
   private final YoPoint2D point;
   
   private final YoOrientation2D orientation;
   
   public YoPose2D(String namePrefix, YoVariableRegistry registry)
   {
      point = new YoPoint2D(String.format("%s_point", namePrefix), registry);
      orientation = new YoOrientation2D(String.format("%s_orientation", namePrefix), registry);
   }
   
   public YoPose2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      point = new YoPoint2D(String.format("%s_point", namePrefix), nameSuffix, registry);
      orientation = new YoOrientation2D(String.format("%s_orientation", namePrefix), nameSuffix, registry);
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
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
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
   public double getYaw()
   {
      return orientation.getYaw();
   }
}
