package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.DynamicReachabilityParameters;

public class AtlasDynamicReachabilityParameters extends DynamicReachabilityParameters
{
   private final boolean runningOnRealRobot;

   public AtlasDynamicReachabilityParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   /** {@inheritDoc} */
   public double getMaximumDesiredKneeBend()
   {
      return 0.45;
   }


   @Override
   /** {@inheritDoc} */
   public double getMinimumSwingDuration()
   {
      return runningOnRealRobot ? 0.7 : 0.3;
   }

   @Override
   /** {@inheritDoc} */
   public double getMinimumTransferDuration()
   {
      return runningOnRealRobot ? 0.3 : 0.2;
   }

   @Override
   /** {@inheritDoc} */
   public double getTotalSwingAdjustmentWeight()
   {
      return 10.0;
   }
}

