package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple2D.Point2DBasicsTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

public class YoPoint2DTest extends Point2DBasicsTest<YoPoint2D>
{
   @Override
   public YoPoint2D createEmptyTuple()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      return new YoPoint2D("testYoPoint2D", registry);
   }

   @Override
   public YoPoint2D createTuple(double x, double y)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoPoint2D point = new YoPoint2D("testYoPoint2D", registry);

      point.set(x, y);

      return point;
   }

   @Override
   public YoPoint2D createRandomTuple(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoPoint2D point = new YoPoint2D("testYoPoint2D", registry);

      point.set(random.nextDouble(), random.nextDouble());

      return point;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}