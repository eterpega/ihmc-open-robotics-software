package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFallDetector
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   // Yo Variables
   private final DoubleYoVariable yoDcmDistanceOutsideSupportPolygon;

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter maxPitchInRad = parameterFactory.createDouble("maxPitchInRad", .5);
   private final DoubleParameter maxRollInRad = parameterFactory.createDouble("maxRollInRad", .5);
   private final DoubleParameter dcmOutsideSupportThreshold = parameterFactory.createDouble("DcmDistanceOutsideSupportPolygonSupportThreshold", .15);

   //Estimation Variables
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final QuadrupedSupportPolygon supportPolygon;

   public enum fallDetectionTypes
   {
      NONE, ROLL_LIMIT, PITCH_LIMIT, PITCH_AND_ROLL_LIMIT, DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT, ALL
   }
   private final EnumYoVariable<fallDetectionTypes> fallDetectionType;

   public QuadrupedFallDetector(QuadrupedTaskSpaceEstimator taskSpaceEstimator, DivergentComponentOfMotionEstimator dcmPositionEstimator, YoVariableRegistry parentRegistry)
   {
      this.taskSpaceEstimator = taskSpaceEstimator;
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      dcmPositionEstimate = new FramePoint();
      this.dcmPositionEstimator = dcmPositionEstimator;
      fallDetectionType = EnumYoVariable.create("fallDetectionTypes", fallDetectionTypes.class, registry);
      fallDetectionType.set(fallDetectionTypes.NONE);
      supportPolygon = new QuadrupedSupportPolygon(taskSpaceEstimates.getSolePosition());
      yoDcmDistanceOutsideSupportPolygon = new DoubleYoVariable("DcmDistanceOutsideSupportPolygon", registry);
      parentRegistry.addChild(registry);
   }

   public boolean detect()
   {
      updateEstimates();
      switch (fallDetectionType.getEnumValue())
      {
      case DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT:
         return detectDcmDistanceOutsideSupportPolygonLimitFailure();
      case ROLL_LIMIT:
         return detectRollLimitFailure();
      case PITCH_LIMIT:
         return detectPitchLimitFailure();
      case PITCH_AND_ROLL_LIMIT:
         return detectPitchAndRollLimitFailure();
      case ALL:
         return detectDcmDistanceOutsideSupportPolygonLimitFailure() || detectPitchAndRollLimitFailure();
      default:
         return false;
      }
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         taskSpaceEstimates.getSolePosition(quadrant).changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(quadrant, taskSpaceEstimates.getSolePosition(quadrant));
      }
   }

   private boolean detectPitchAndRollLimitFailure()
   {
      return detectPitchLimitFailure() || detectRollLimitFailure();
   }

   private boolean detectPitchLimitFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      return Math.abs(taskSpaceEstimates.getBodyOrientation().getPitch()) > maxPitchInRad.get();
   }

   private boolean detectRollLimitFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      return Math.abs(taskSpaceEstimates.getBodyOrientation().getRoll()) > maxRollInRad.get();
   }

   private boolean detectDcmDistanceOutsideSupportPolygonLimitFailure()
   {
      yoDcmDistanceOutsideSupportPolygon.set(supportPolygon.getDistanceOutside2d(dcmPositionEstimate));
      return supportPolygon.getDistanceOutside2d(dcmPositionEstimate) > dcmOutsideSupportThreshold.get();
   }
}
