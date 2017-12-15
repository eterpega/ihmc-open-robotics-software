package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple4D.QuaternionBasicsTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

public class YoQuaternionTest extends QuaternionBasicsTest<YoQuaternion>
{
   @Override
   public YoQuaternion createEmptyTuple()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      return new YoQuaternion("testYoQuaternion", registry);
   }

   @Override
   public YoQuaternion createTuple(double x, double y, double z, double s)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoQuaternion quaternion = new YoQuaternion("testYoQuaternion", registry);

      quaternion.setUnsafe(x, y, z, s);

      return quaternion;
   }

   @Override
   public YoQuaternion createRandomTuple(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoQuaternion quaternion = new YoQuaternion("testYoQuaternion", registry);

      quaternion.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

      return quaternion;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}
