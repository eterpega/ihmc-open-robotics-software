package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple2D.Vector2DBasicsTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

public class YoVector2DTest extends Vector2DBasicsTest<YoVector2D>
{
   @Override
   public YoVector2D createEmptyTuple()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      return new YoVector2D("testYoVector2D", registry);
   }

   @Override
   public YoVector2D createTuple(double x, double y)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoVector2D vector = new YoVector2D("testYoVector2D", registry);

      vector.set(x, y);

      return vector;
   }

   @Override
   public YoVector2D createRandomTuple(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoVector2D vector = new YoVector2D("testYoVector2D", registry);

      vector.set(random.nextDouble(), random.nextDouble());

      return vector;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}
