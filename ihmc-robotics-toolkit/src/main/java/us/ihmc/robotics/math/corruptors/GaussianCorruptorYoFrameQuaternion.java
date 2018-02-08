package us.ihmc.robotics.math.corruptors;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GaussianCorruptorYoFrameQuaternion extends YoFrameQuaternion implements ProcessingYoVariable
{
   private final Random random;
   private final Vector3DReadOnly standardDeviation;
   private final QuaternionReadOnly input;

   private static Vector3DReadOnly convertStandardDeviationToVector(DoubleProvider standardDeviation)
   {
      return new Vector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return standardDeviation.getValue();
         }

         @Override
         public double getY()
         {
            return standardDeviation.getValue();
         }

         @Override
         public double getZ()
         {
            return standardDeviation.getValue();
         }
      };
   }

   public GaussianCorruptorYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation,
                                             ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, random, convertStandardDeviationToVector(standardDeviation), null, referenceFrame);
   }

   public GaussianCorruptorYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, Random random, DoubleProvider standardDeviation,
                                             FrameQuaternionReadOnly inputVariable)
   {
      this(namePrefix, nameSuffix, registry, random, convertStandardDeviationToVector(standardDeviation), inputVariable, inputVariable.getReferenceFrame());
   }

   public GaussianCorruptorYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, Random random,
                                             Vector3DReadOnly standardDeviation, ReferenceFrame referenceFrame)
   {
      this(namePrefix, nameSuffix, registry, random, standardDeviation, null, referenceFrame);
   }

   public GaussianCorruptorYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, Random random,
                                             Vector3DReadOnly standardDeviation, FrameQuaternionReadOnly inputVariable)
   {
      this(namePrefix, nameSuffix, registry, random, standardDeviation, inputVariable, inputVariable.getReferenceFrame());
   }

   private GaussianCorruptorYoFrameQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry, Random random,
                                              Vector3DReadOnly standardDeviation, QuaternionReadOnly inputVariable, ReferenceFrame referenceFrame)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);
      this.random = random;
      input = inputVariable;

      if (standardDeviation == null)
         standardDeviation = new YoFrameVector(namePrefix + "StandardDeviation" + nameSuffix, referenceFrame, registry);
      this.standardDeviation = standardDeviation;
   }

   @Override
   public void update()
   {
      if (input == null)
      {
         throw new NullPointerException("GaussianCorruptorYoFrameQuaternion must be constructed with a non null "
               + "input variable to call update(), otherwise use update(FrameQuaternionReadOnly)");
      }

      update(input);
   }

   public void update(FrameQuaternionReadOnly input)
   {
      checkReferenceFrameMatch(input);
      update((QuaternionReadOnly) input);
   }

   private final Vector3D noiseRotationVector = new Vector3D();

   public void update(QuaternionReadOnly input)
   { // TODO Not sure if that's the proper way to generate axis-dependent noise.
      noiseRotationVector.set(standardDeviation);
      noiseRotationVector.scale(random.nextGaussian(), random.nextGaussian(), random.nextGaussian());
      set(noiseRotationVector);
      preMultiply(input);
   }
}
