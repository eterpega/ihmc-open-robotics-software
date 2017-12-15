package us.ihmc.robotics.math;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class YoPoint3D implements Point3DBasics, GeometryObject<YoPoint3D>
{
   /** The x-coordinate. */
   private final YoDouble x;
   /** The y-coordinate. */
   private final YoDouble y;
   /** The z-coordinate. */
   private final YoDouble z;

   /**
    * Creates a new yo point using the given variables.
    *
    * @param xVariable the x-coordinate variable.
    * @param yVariable the y-coordinate variable.
    * @param zVariable the z-coordinate variable.
    */
   public YoPoint3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable)
   {
      x = xVariable;
      y = yVariable;
      z = zVariable;
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoPoint3D(String namePrefix, YoVariableRegistry registry)
   {
      this(namePrefix, "", registry);
   }
   
   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoPoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
   }
   
   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z.set(z);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(YoPoint3D other, double epsilon)
   {
      return Point3DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoPoint3D other, double epsilon)
   {
      return Point3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoPoint3D other)
   {
      Point3DBasics.super.set(other);
   }
}
