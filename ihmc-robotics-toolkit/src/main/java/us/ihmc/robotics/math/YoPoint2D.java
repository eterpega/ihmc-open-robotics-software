package us.ihmc.robotics.math;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoPoint2D implements Point2DBasics, GeometryObject<YoPoint2D>
{
   /** The x-coordinate. */
   private final YoDouble x;
   /** The y-coordinate. */
   private final YoDouble y;

   /**
    * Creates a new yo point using the given variables.
    *
    * @param xVariable the x-coordinate variable.
    * @param yVariable the y-coordinate variable.
    */
   public YoPoint2D(YoDouble xVariable, YoDouble yVariable)
   {
      x = xVariable;
      y = yVariable;
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables
    * to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry the registry to register child variables to.
    */
   public YoPoint2D(String namePrefix, YoVariableRegistry registry)
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
   public YoPoint2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
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
   public boolean epsilonEquals(YoPoint2D other, double epsilon)
   {
      return Point2DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoPoint2D other, double epsilon)
   {
      return Point2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoPoint2D other)
   {
      Point2DBasics.super.set(other);
   }
}
