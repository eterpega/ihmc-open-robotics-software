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

public class ThreeDoFWaypointSwingFootTrajectory extends AdjustableThreeDoFSwingFootTrajectory
{
   private final MultipleWaypointsPositionTrajectoryGenerator trajectory;
   private boolean initialized;

   public ThreeDoFWaypointSwingFootTrajectory(String namePrefix, YoVariableRegistry registry)
   {
      trajectory = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix + "SwingFoot", true, ReferenceFrame.getWorldFrame(), registry);
      initialized = false;
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
      super.initializeAdjustmentTrajectory(initialPosition, finalPosition, startTime, endTime);

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

      initialized = true;
      computeTrajectory(0);
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