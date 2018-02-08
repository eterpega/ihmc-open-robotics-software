package us.ihmc.robotics.math.corruptors;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createXName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createYName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createZName;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.robotics.math.frames.YoFrameTuple;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GaussianCorruptorYoFrameVector3D extends YoFrameVector implements ProcessingYoVariable
{
   private final GaussianCorruptorYoVariable x, y, z;

   private GaussianCorruptorYoFrameVector3D(GaussianCorruptorYoVariable x, GaussianCorruptorYoVariable y, GaussianCorruptorYoVariable z,
                                            ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, DoubleProvider standardDeviation,
                                                                                         ReferenceFrame referenceFrame)
   {
      GaussianCorruptorYoVariable x = new GaussianCorruptorYoVariable(createXName(namePrefix, nameSuffix), registry, random, standardDeviation);
      GaussianCorruptorYoVariable y = new GaussianCorruptorYoVariable(createYName(namePrefix, nameSuffix), registry, random, standardDeviation);
      GaussianCorruptorYoVariable z = new GaussianCorruptorYoVariable(createZName(namePrefix, nameSuffix), registry, random, standardDeviation);

      return new GaussianCorruptorYoFrameVector3D(x, y, z, referenceFrame);
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, DoubleProvider standardDeviation, YoFrameTuple input)
   {
      GaussianCorruptorYoVariable x, y, z;
      x = new GaussianCorruptorYoVariable(createXName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoX());
      y = new GaussianCorruptorYoVariable(createYName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoY());
      z = new GaussianCorruptorYoVariable(createZName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, input.getReferenceFrame());
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, Vector3DReadOnly standardDeviation,
                                                                                         ReferenceFrame referenceFrame)
   {
      GaussianCorruptorYoVariable x = new GaussianCorruptorYoVariable(createXName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getX());
      GaussianCorruptorYoVariable y = new GaussianCorruptorYoVariable(createYName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getY());
      GaussianCorruptorYoVariable z = new GaussianCorruptorYoVariable(createZName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, referenceFrame);
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, Vector3DReadOnly standardDeviation, YoFrameTuple input)
   {
      GaussianCorruptorYoVariable x, y, z;
      x = new GaussianCorruptorYoVariable(createXName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getX(), input.getYoX());
      y = new GaussianCorruptorYoVariable(createYName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getY(), input.getYoY());
      z = new GaussianCorruptorYoVariable(createZName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getZ(), input.getYoZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, input.getReferenceFrame());
   }

   @Override
   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xInput, double yInput, double zInput)
   {
      x.update(xInput);
      y.update(yInput);
      z.update(zInput);
   }

   public void update(Tuple3DReadOnly input)
   {
      x.update(input.getX());
      y.update(input.getY());
      z.update(input.getZ());
   }

   public void update(FrameTuple3DReadOnly inpput)
   {
      checkReferenceFrameMatch(inpput);
      x.update(inpput.getX());
      y.update(inpput.getY());
      z.update(inpput.getZ());
   }
}
