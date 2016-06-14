package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class QuadrupedFallDetector
{
   private final YoVariableRegistry registry;

   // parameters
   private final ParameterFactory parameterFactory;
   private final DoubleParameter maxPitchInRad;
   private final DoubleParameter maxRollInRad;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   public enum fallDetectionTypes{
      ROLL_TILT, PITCH_TILT, ROLL_AND_PITCH_TILT
   }
   private final EnumYoVariable<fallDetectionTypes> fallDetectionType;

   public QuadrupedFallDetector(YoVariableRegistry registryInput, QuadrupedTaskSpaceEstimator taskSpaceEstimatorInput)
   {
      taskSpaceEstimator = taskSpaceEstimatorInput;
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      registry = registryInput;
      fallDetectionType = EnumYoVariable.create("fallDetectionTypes", fallDetectionTypes.class, registry);
      fallDetectionType.set(fallDetectionTypes.ROLL_AND_PITCH_TILT);
      parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
      maxPitchInRad = parameterFactory.createDouble("maxPitchInRad", .5);
      maxRollInRad = parameterFactory.createDouble("maxRollInRad", .5);
   }

   public boolean detect(){
      return detectTiltFailure();
   }

    public boolean detectTiltFailure()
   {
      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      if(fallDetectionType.getEnumValue() == fallDetectionTypes.PITCH_TILT || fallDetectionType.getEnumValue() == fallDetectionTypes.ROLL_AND_PITCH_TILT){
         if (Math.abs(taskSpaceEstimates.getBodyOrientation().getPitch()) > maxPitchInRad.get())
         {
            return true;
         }
      }
      if(fallDetectionType.getEnumValue() == fallDetectionTypes.ROLL_TILT || fallDetectionType.getEnumValue() == fallDetectionTypes.ROLL_AND_PITCH_TILT){
         if (Math.abs(taskSpaceEstimates.getBodyOrientation().getRoll()) > maxRollInRad.get())
         {
            return true;
         }
      }
      return false;
   }

}
