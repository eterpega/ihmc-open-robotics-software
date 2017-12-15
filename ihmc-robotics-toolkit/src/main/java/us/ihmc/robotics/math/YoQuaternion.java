package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoQuaternion implements QuaternionBasics
{
   private final YoDouble x;
   private final YoDouble y;
   private final YoDouble z;
   private final YoDouble s;
   
   public YoQuaternion(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, YoDouble sVariable)
   {
      x = xVariable;
      y = yVariable;
      z = zVariable;
      s = sVariable;
   }

   public YoQuaternion(String namePrefix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, ""), registry);
      y = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, ""), registry);
      z = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, ""), registry);
      s = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, ""), registry);
   }

   public YoQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, nameSuffix), registry);
      s = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, nameSuffix), registry);
   }
   
   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      this.x.set(qx);
      this.y.set(qy);
      this.z.set(qz);
      this.s.set(qs);
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

   @Override
   public double getZ()
   {
      return z.getDoubleValue();
   }

   @Override
   public double getS()
   {
      return s.getDoubleValue();
   }
}
