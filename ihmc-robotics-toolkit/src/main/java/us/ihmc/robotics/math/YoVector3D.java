package us.ihmc.robotics.math;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVector3D implements Vector3DBasics, GeometryObject<YoVector3D>
{
   /** The x-component. */
   private final YoDouble x;
   /** The y-component. */
   private final YoDouble y;
   /** The z-component. */
   private final YoDouble z;

   /**
    * Creates a new yo vector using the given variables.
    *
    * @param xVariable the x-component variable.
    * @param yVariable the y-component variable.
    * @param zVariable the z-component variable.
    */
   public YoVector3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable)
   {
      x = xVariable;
      y = yVariable;
      z = zVariable;
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoVector3D(String namePrefix, YoVariableRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
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
   public boolean epsilonEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoVector3D other)
   {
      Vector3DBasics.super.set(other);
   }
}
