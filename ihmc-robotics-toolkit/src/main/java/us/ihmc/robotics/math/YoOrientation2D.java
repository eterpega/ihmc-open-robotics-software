package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// TODO: implements Orientation2DBasics, GeometryObject<YoOrientation2D>
public class YoOrientation2D
{
   private final YoDouble yaw;

   public YoOrientation2D(YoDouble yawVariable)
   {
      yaw = yawVariable;
   }

   public YoOrientation2D(String namePrefix, YoVariableRegistry registry)
   {
      yaw = new YoDouble(String.format("%s_yaw", namePrefix), registry);
   }

   public YoOrientation2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      yaw = new YoDouble(String.format("%s_yaw_%s", namePrefix, nameSuffix), registry);
   }
   
   //@Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw);
   }

   //@Override
   public double getYaw()
   {
      return yaw.getDoubleValue();
   }
}
