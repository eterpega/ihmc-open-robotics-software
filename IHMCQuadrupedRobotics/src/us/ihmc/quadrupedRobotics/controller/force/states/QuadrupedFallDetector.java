package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.DivergentComponentOfMotionEstimator;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFallDetector
{
   private final YoVariableRegistry registry;

   // parameters
   private final ParameterFactory parameterFactory;
   private final DoubleParameter maxPitchInRad;
   private final DoubleParameter maxRollInRad;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrupedSupportPolygon supportPolygon;
   private final DoubleYoVariable yoDCMOutside;
   private final YoFramePoint yoSupportPolygonCentroid;
   private final DoubleParameter DCMOutsideSupportThreshold;

   public enum fallDetectionTypes
   {
      NONE, ROLL_TILT, PITCH_TILT, ROLL_AND_PITCH_TILT, DCM_INSIDE, ALL
   }

   private final EnumYoVariable<fallDetectionTypes> fallDetectionType;

   public QuadrupedFallDetector(YoVariableRegistry registryInput, QuadrupedForceControllerToolbox controllerToolbox)
   {
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      dcmPositionEstimate = new FramePoint();
      dcmPositionEstimator = controllerToolbox.getDcmPositionEstimator();
      registry = registryInput;
      fallDetectionType = EnumYoVariable.create("fallDetectionTypes", fallDetectionTypes.class, registry);
      fallDetectionType.set(fallDetectionTypes.NONE);
      parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
      maxPitchInRad = parameterFactory.createDouble("maxPitchInRad", .5);
      maxRollInRad = parameterFactory.createDouble("maxRollInRad", .5);
      referenceFrames = controllerToolbox.getReferenceFrames();
      supportPolygon = new QuadrupedSupportPolygon(taskSpaceEstimates.getSolePosition());
      yoDCMOutside = new DoubleYoVariable("DCMoutside", registry);
      yoSupportPolygonCentroid = new YoFramePoint("supportPolygonCentriod", ReferenceFrame.getWorldFrame(), registry);
      DCMOutsideSupportThreshold = parameterFactory.createDouble("DCMOutsideSupportThreshold", .15);
   }

   public boolean detect()
   {
      updateEstimates();

      switch (fallDetectionType.getEnumValue())
      {
      case DCM_INSIDE:
         return detectDCMFailure();
      case ROLL_TILT:
         return detectRollTiltFailure();
      case PITCH_TILT:
         return detectPitchTiltFailure();
      case ROLL_AND_PITCH_TILT:
         return detectTiltFailure();
      case ALL:
         return detectDCMFailure() || detectTiltFailure();
      default:
         return false;
      }
   }

   public void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         taskSpaceEstimates.getSolePosition(quadrant).changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(quadrant, taskSpaceEstimates.getSolePosition(quadrant));
      }
   }

   public boolean detectTiltFailure()
   {
      return detectPitchTiltFailure() || detectRollTiltFailure();
   }

   public boolean detectPitchTiltFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      if (Math.abs(taskSpaceEstimates.getBodyOrientation().getPitch()) > maxPitchInRad.get())
      {
         return true;
      }
      return false;
   }

   public boolean detectRollTiltFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      if (Math.abs(taskSpaceEstimates.getBodyOrientation().getRoll()) > maxRollInRad.get())
      {
         return true;
      }
      return false;
   }

   public boolean detectDCMFailure()
   {
      yoDCMOutside.set(supportPolygon.getDistanceOutside2d(dcmPositionEstimate));
      FramePoint tempFramePoint = new FramePoint();
      supportPolygon.getCentroid(tempFramePoint);
      yoSupportPolygonCentroid.set(tempFramePoint);
      if (supportPolygon.getDistanceOutside2d(dcmPositionEstimate) > DCMOutsideSupportThreshold.get())
      {
         return true;
      }
      return false;

   }
}
