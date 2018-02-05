package us.ihmc.llama.model;

import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;

public class LlamaSensorInformation implements QuadrupedSensorInformation
{
   private static final String[] imuNames = new String[] {"body_imu"};
   
   @Override
   public String[] getImuNames()
   {
      return imuNames;
   }
}
