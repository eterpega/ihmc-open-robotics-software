package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple3D.Point3DBasicsTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

public class YoPoint3DTest extends Point3DBasicsTest<YoPoint3D>
{
   @Override
   public YoPoint3D createEmptyTuple()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      return new YoPoint3D("testYoPoint3D", registry);
   }

   @Override
   public YoPoint3D createTuple(double x, double y, double z)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoPoint3D point = new YoPoint3D("testYoPoint3D", registry);

      point.set(x, y, z);

      return point;
   }

   @Override
   public YoPoint3D createRandomTuple(Random random)
   {
      YoVariableRegistry registry = new YoVariableRegistry("testYoVariableRegistry");

      YoPoint3D point = new YoPoint3D("testYoPoint3D", registry);

      point.set(random.nextDouble(), random.nextDouble(), random.nextDouble());

      return point;
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-12;
   }
}