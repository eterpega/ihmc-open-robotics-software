package us.ihmc.robotics.math;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVector2D implements Vector2DBasics, GeometryObject<YoVector2D>
{
   /** The x-component. */
   private final YoDouble x;
   /** The y-component. */
   private final YoDouble y;

   /**
    * Creates a new yo vector using the given variables.
    *
    * @param xVariable the x-component variable.
    * @param yVariable the y-component variable.
    */
   public YoVector2D(YoDouble xVariable, YoDouble yVariable)
   {
      x = xVariable;
      y = yVariable;
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoVector2D(String namePrefix, YoVariableRegistry registry)
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
   public YoVector2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
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
   public boolean epsilonEquals(YoVector2D other, double epsilon)
   {
      return Vector2DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoVector2D other, double epsilon)
   {
      return Vector2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoVector2D other)
   {
      Vector2DBasics.super.set(other);
   }
}
