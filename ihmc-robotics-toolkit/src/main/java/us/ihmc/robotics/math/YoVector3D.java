package us.ihmc.robotics.math;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVector3D implements Vector3DBasics, GeometryObject<YoVector3D>
{
   private final YoDouble x;
   private final YoDouble y;
   private final YoDouble z;

   public YoVector3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable)
   {
      this.x = xVariable;
      this.y = yVariable;
      this.z = zVariable;
   }

   public YoVector3D(String namePrefix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, ""), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, ""), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, ""), registry);
   }

   public YoVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
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
   public void setZ(double z)
   {
      this.z.set(z);
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
   public boolean epsilonEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public void set(YoVector3D other)
   {
      Vector3DBasics.super.set(other);
   }
}
