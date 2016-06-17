package us.ihmc.quadrupedRobotics.planning.trajectory;

import java.util.List;

import us.ihmc.quadrupedRobotics.planning.QuadrupedStepWaypoint;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class ThreeDoFWaypointSwingFootTrajectory
{
   final private MinimumJerkTrajectory xTrajectoryAdjustment;
   final private MinimumJerkTrajectory yTrajectoryAdjustment;
   final private MinimumJerkTrajectory zTrajectoryAdjustment;
   final private FramePoint initialPosition;
   final private FramePoint finalPosition;
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private TimeInterval timeInterval;
   private TimeInterval timeIntervalOfAdjustment;
   private boolean initialized;

   private final MultipleWaypointsPositionTrajectoryGenerator trajectory;

   public ThreeDoFWaypointSwingFootTrajectory(String namePrefix, YoVariableRegistry registry)
   {
      trajectory = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix + "SwingFoot", true, ReferenceFrame.getWorldFrame(), registry);

      xTrajectoryAdjustment = new MinimumJerkTrajectory();
      yTrajectoryAdjustment = new MinimumJerkTrajectory();
      zTrajectoryAdjustment = new MinimumJerkTrajectory();
      initialPosition = new FramePoint();
      finalPosition = new FramePoint();
      position = new FramePoint();
      velocity = new FrameVector();
      acceleration = new FrameVector();
      referenceFrame = ReferenceFrame.getWorldFrame();
      timeInterval = new TimeInterval();
      timeIntervalOfAdjustment = new TimeInterval();
      initialized = false;
   }

   public double getStartTime()
   {
      return timeInterval.getStartTime();
   }

   public double getEndTime()
   {
      return timeInterval.getEndTime();
   }

   public void getPosition(FramePoint position)
   {
      position.setIncludingFrame(this.position);
   }

   public void getVelocity(FrameVector velocity)
   {
      velocity.setIncludingFrame(this.velocity);
   }

   public void getAcceleration(FrameVector acceleration)
   {
      acceleration.setIncludingFrame(this.acceleration);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void initializeTrajectory(FramePoint initialPosition, List<QuadrupedStepWaypoint> waypoints, int waypointCount, FramePoint finalPosition, TimeInterval timeInterval)
   {
      initializeTrajectory(initialPosition, waypoints, waypointCount, finalPosition, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public void initializeTrajectory(FramePoint initialPosition, List<QuadrupedStepWaypoint> waypoints, int waypointCount, FramePoint finalPosition, double duration)
   {
      initializeTrajectory(initialPosition, waypoints, waypointCount, finalPosition, 0, duration);
   }

   public void initializeTrajectory(FramePoint initialPosition, List<QuadrupedStepWaypoint> waypoints, int waypointCount, FramePoint finalPosition, double startTime, double endTime)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.finalPosition.setIncludingFrame(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();
      timeInterval.setInterval(startTime, endTime);
      timeIntervalOfAdjustment.setInterval(startTime, endTime);

      trajectory.clear(referenceFrame);

      trajectory.appendWaypoint(0.0, initialPosition, new FrameVector()); // TODO: Start velocity
      for (int i = 0; i < waypointCount; i++)
      {
         QuadrupedStepWaypoint waypoint = waypoints.get(i);
         System.out.println(waypoint);

         if (waypoint.timeInStep < 0.0 || waypoint.timeInStep > timeInterval.getDuration())
            throw new RuntimeException("waypoint time is out of bounds of step");

         trajectory.appendWaypoint(waypoint.timeInStep, waypoint.position, waypoint.velocity);
      }
      trajectory.appendWaypoint(timeInterval.getDuration(), finalPosition, new FrameVector()); // TODO: End velocity
      trajectory.initialize();

      xTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      yTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      zTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      initialized = true;

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
      computeTrajectory(0);
   }

   public void adjustTrajectory(FramePoint finalPosition, double currentTime)
   {
      computeTrajectory(currentTime);
      timeIntervalOfAdjustment.setStartTime(Math.min(Math.max(currentTime, timeInterval.getStartTime()), timeInterval.getEndTime()));
      xTrajectoryAdjustment.setMoveParameters(xTrajectoryAdjustment.getPosition(), xTrajectoryAdjustment.getVelocity(), xTrajectoryAdjustment.getAcceleration(), finalPosition.getX() - this.finalPosition.getX(), (finalPosition.getX() - this.finalPosition.getX()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      yTrajectoryAdjustment.setMoveParameters(yTrajectoryAdjustment.getPosition(), yTrajectoryAdjustment.getVelocity(), yTrajectoryAdjustment.getAcceleration(), finalPosition.getY() - this.finalPosition.getY(), (finalPosition.getY() - this.finalPosition.getY()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      zTrajectoryAdjustment.setMoveParameters(zTrajectoryAdjustment.getPosition(), zTrajectoryAdjustment.getVelocity(), zTrajectoryAdjustment.getAcceleration(), finalPosition.getZ() - this.finalPosition.getZ(), 0, 0, timeIntervalOfAdjustment.getDuration());
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("parameters must be initialized before computing trajectory");

      trajectory.compute(currentTime - timeInterval.getStartTime());

      xTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      yTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      zTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());

      trajectory.getPosition(position);
      trajectory.getVelocity(velocity);
      trajectory.getAcceleration(acceleration);

      position.add(xTrajectoryAdjustment.getPosition(), yTrajectoryAdjustment.getPosition(), zTrajectoryAdjustment.getPosition());
      velocity.add(xTrajectoryAdjustment.getVelocity(), yTrajectoryAdjustment.getVelocity(), zTrajectoryAdjustment.getVelocity());
      acceleration.add(xTrajectoryAdjustment.getAcceleration(), yTrajectoryAdjustment.getAcceleration(), zTrajectoryAdjustment.getAcceleration());
   }
}