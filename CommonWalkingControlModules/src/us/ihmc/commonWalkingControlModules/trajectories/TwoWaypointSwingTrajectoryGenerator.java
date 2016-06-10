package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class TwoWaypointSwingTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;

   private final ReferenceFrame trajectoryFrame;

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final DoubleYoVariable adjustedTimeIntoStep;
   private final BooleanYoVariable isDone;
   private final EnumYoVariable<TrajectoryType> trajectoryType;

   private final FramePoint initialPosition = new FramePoint();
   private final FrameVector initialVelocity = new FrameVector();

   private final FramePoint finalPosition = new FramePoint();
   private final FrameVector finalVelocity = new FrameVector();

   private final FramePoint waypointAPosition = new FramePoint();
   private final FrameVector waypointAVelocity = new FrameVector();
   private final DoubleYoVariable waypointATime;

   private final FramePoint waypointBPosition = new FramePoint();
   private final FrameVector waypointBVelocity = new FrameVector();
   private final DoubleYoVariable waypointBTime;

   private final YoPolynomial trajectoryPolynomialX;
   private final YoPolynomial trajectoryPolynomialY;
   private final YoPolynomial trajectoryPolynomialZ;
   private final YoPolynomial trajectoryPolynomialZMid;
   private final FrameVector tempVector = new FrameVector();

   private final int markers = 20;
   private final BagOfBalls trajectoryViz;
   private final FramePoint ballPosition = new FramePoint();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   public TwoWaypointSwingTrajectoryGenerator(String namePrefix, ReferenceFrame trajectoryFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.trajectoryFrame = trajectoryFrame;

      stepTime = new DoubleYoVariable(namePrefix + "StepTime", registry);
      timeIntoStep = new DoubleYoVariable(namePrefix + "TimeIntoStep", registry);
      adjustedTimeIntoStep = new DoubleYoVariable(namePrefix + "AdjustedTimeIntoStep", registry);
      isDone = new BooleanYoVariable(namePrefix + "IsDone", registry);
      trajectoryType = new EnumYoVariable<>(namePrefix + "TrajectoryType", registry, TrajectoryType.class);

      trajectoryPolynomialX = new YoPolynomial("", 12, new YoVariableRegistry(""));
      trajectoryPolynomialY = new YoPolynomial("", 12, new YoVariableRegistry(""));
      trajectoryPolynomialZ = new YoPolynomial("", 12, new YoVariableRegistry(""));
      trajectoryPolynomialZMid = new YoPolynomial("", 2, new YoVariableRegistry(""));

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", trajectoryFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", trajectoryFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", trajectoryFrame, registry);

      waypointATime = new DoubleYoVariable(namePrefix + "WaypointATime", registry);
      waypointBTime = new DoubleYoVariable(namePrefix + "WaypointBTime", registry);

      if (yoGraphicsListRegistry != null)
      {
         trajectoryViz = new BagOfBalls(markers, 0.01, namePrefix + "Trajectory", new YoVariableRegistry(""), yoGraphicsListRegistry);
      }
      else
      {
         trajectoryViz = null;
      }
   }

   public void setStepTime(double stepTime)
   {
      this.stepTime.set(stepTime);
   }

   public void setInitialConditions(FramePoint initialPosition, FrameVector initialVelocity)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialPosition.changeFrame(trajectoryFrame);
      this.initialVelocity.changeFrame(trajectoryFrame);

   }

   public void setFinalConditions(FramePoint finalPosition, FrameVector finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalPosition.changeFrame(trajectoryFrame);
      this.finalVelocity.changeFrame(trajectoryFrame);
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType.set(trajectoryType);
   }

   public void informDone()
   {
      desiredPosition.setToZero(true);
      desiredVelocity.setToZero(true);
      desiredAcceleration.setToZero(true);
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      isDone.set(false);

      // TODO: get this from parameters and do checks on swing height
      double swingHeight = 0.1;
      double[] proportions = {0.4, 0.6};

      waypointAPosition.interpolate(initialPosition, finalPosition, proportions[0]);
      waypointBPosition.interpolate(initialPosition, finalPosition, proportions[1]);

      switch (trajectoryType.getEnumValue())
      {
      case OBSTACLE_CLEARANCE:
         double maxZ = Math.max(initialPosition.getZ(), finalPosition.getZ());
         waypointAPosition.setZ(maxZ + swingHeight);
         waypointBPosition.setZ(maxZ + swingHeight);
         break;
      case DEFAULT:
         waypointAPosition.add(0.0, 0.0, swingHeight);
         waypointBPosition.add(0.0, 0.0, swingHeight);
         break;
      default:
         throw new RuntimeException("Trajectory type not implemented");
      }

      // --- Adjust times according to trajectory length
//      tempVector.sub(waypointAPosition, initialPosition);
//      double d1 = tempVector.length();
//      tempVector.sub(waypointBPosition, waypointAPosition);
//      double d2 = tempVector.length();
//      tempVector.sub(finalPosition, waypointBPosition);
//      double d3 = tempVector.length();
//      double d = d1 + d2 + d3;
//
//      proportions[0] = d1/d;
//      proportions[1] = (d1+d2)/d;
//
//      System.out.println("Proportions: [" + proportions[0] + ", " + proportions[1] + "]");
      // ---

      double trajectoryTime = stepTime.getDoubleValue();
      double waypointATime = trajectoryTime * proportions[0];
      double waypointBTime = trajectoryTime * proportions[1];

      waypointAVelocity.sub(waypointBPosition, waypointAPosition);
      waypointAVelocity.scale(1.0/(waypointBTime - waypointATime));
      waypointBVelocity.setIncludingFrame(waypointAVelocity);

      double[] times = {0.0, waypointATime, waypointBTime, trajectoryTime};
      double[] zero = {0.0, 0.0, 0.0, 0.0};

      double[] positionsX = {initialPosition.getX(), waypointAPosition.getX(), waypointBPosition.getX(), finalPosition.getX()};
      double[] velocitiesX = {initialVelocity.getX(), waypointAVelocity.getX(), waypointBVelocity.getX(), finalVelocity.getX()};
      trajectoryPolynomialX.setQuintic(times[0], times[3], positionsX[0], velocitiesX[0], 0.0, positionsX[3], velocitiesX[3], 0.0);

      double[] positionsY = {initialPosition.getY(), waypointAPosition.getY(), waypointBPosition.getY(), finalPosition.getY()};
      double[] velocitiesY = {initialVelocity.getY(), waypointAVelocity.getY(), waypointBVelocity.getY(), finalVelocity.getY()};
      trajectoryPolynomialY.setQuintic(times[0], times[3], positionsY[0], velocitiesY[0], 0.0, positionsY[3], velocitiesY[3], 0.0);

      double[] positionsZ = {initialPosition.getZ(), waypointAPosition.getZ(), waypointBPosition.getZ(), finalPosition.getZ()};
      double[] velocitiesZ = {initialVelocity.getZ(), waypointAVelocity.getZ(), waypointBVelocity.getZ(), finalVelocity.getZ()};
      trajectoryPolynomialZ.setOrder11(times, positionsZ, velocitiesZ, zero);
      trajectoryPolynomialZMid.setLinear(times[1], times[2], positionsZ[1], positionsZ[2]);

      this.waypointATime.set(waypointATime);
      this.waypointBTime.set(waypointBTime);
      visualize();
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      for (int i = 0; i < markers; i++)
      {
         double percent = (double) i / (double) markers;
         double time = percent * stepTime.getDoubleValue();
         compute(time);
         getPosition(ballPosition);
         trajectoryViz.setBall(ballPosition, i);
      }
   }

   @Override
   public void compute(double time)
   {
      double trajectoryTime = stepTime.getDoubleValue();
      isDone.set(time >= trajectoryTime);

      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime);
      timeIntoStep.set(time);

      // --- Adjust times according to trajectory length
      tempVector.sub(waypointAPosition, initialPosition);
      double d1 = tempVector.length();
      tempVector.sub(waypointBPosition, waypointAPosition);
      double d2 = tempVector.length();
      tempVector.sub(finalPosition, waypointBPosition);
      double d3 = tempVector.length();
      double d = d1 + d2 + d3;

      double waypointATimeAdj = trajectoryTime * d1/d;
      double waypointBTimeAdj = trajectoryTime * (d1+d2)/d;

      if (time < waypointATimeAdj)
      {
         double inclination = waypointATime.getDoubleValue() / waypointATimeAdj;
         double offset = 0.0;
         adjustedTimeIntoStep.set(offset + time * inclination);
      }
      else if (time < waypointBTimeAdj)
      {
         double inclination = (waypointBTime.getDoubleValue() - waypointATime.getDoubleValue()) / (waypointBTimeAdj - waypointATimeAdj);
         double offset = waypointATime.getDoubleValue() - waypointATimeAdj * inclination;
         adjustedTimeIntoStep.set(offset + time * inclination);
      }
      else
      {
         double inclination = (trajectoryTime - waypointBTime.getDoubleValue()) / (trajectoryTime - waypointBTimeAdj);
         double offset = waypointBTime.getDoubleValue() - waypointBTimeAdj * inclination;
         adjustedTimeIntoStep.set(offset + time * inclination);
      }

//      time = adjustedTimeIntoStep.getDoubleValue();
      // ---

      trajectoryPolynomialX.compute(time);
      desiredPosition.setX(trajectoryPolynomialX.getPosition());
      desiredVelocity.setX(trajectoryPolynomialX.getVelocity());
      desiredAcceleration.setX(trajectoryPolynomialX.getAcceleration());

      trajectoryPolynomialY.compute(time);
      desiredPosition.setY(trajectoryPolynomialY.getPosition());
      desiredVelocity.setY(trajectoryPolynomialY.getVelocity());
      desiredAcceleration.setY(trajectoryPolynomialY.getAcceleration());

      if (time < waypointATime.getDoubleValue() || time > waypointBTime.getDoubleValue())
      {
         trajectoryPolynomialZ.compute(time);
         desiredPosition.setZ(trajectoryPolynomialZ.getPosition());
         desiredVelocity.setZ(trajectoryPolynomialZ.getVelocity());
         desiredAcceleration.setZ(trajectoryPolynomialZ.getAcceleration());
      }
      else
      {
         trajectoryPolynomialZMid.compute(time);
         desiredPosition.setZ(trajectoryPolynomialZMid.getPosition());
         desiredVelocity.setZ(trajectoryPolynomialZMid.getVelocity());
         desiredAcceleration.setZ(trajectoryPolynomialZMid.getAcceleration());
      }
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub

   }

}
