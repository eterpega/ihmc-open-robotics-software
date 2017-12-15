package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPoint2D implements Point2DBasics
{
   private final YoDouble x;
   private final YoDouble y;
   
   public YoPoint2D(YoDouble xVariable, YoDouble yVariable)
   {
      this.x = xVariable;
      this.y = yVariable;
   }

   public YoPoint2D(String namePrefix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, ""), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, ""), registry);
   }

   public YoPoint2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
   }
   
   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   @Override
   public double getX()
   {
      return x.getDoubleValue();
   }

   @Override
   public double getY()
   {
      return y.getDoubleValue();
   }
}
