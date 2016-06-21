package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class DefaultThreeDoFSwingFootTrajectory extends AdjustableThreeDoFSwingFootTrajectory
{
   private final MinimumJerkTrajectory xTrajectory;
   private final MinimumJerkTrajectory yTrajectory;
   private final ParabolicTrajectory zTrajectory;
   private boolean initialized;

   public DefaultThreeDoFSwingFootTrajectory()
   {
      xTrajectory = new MinimumJerkTrajectory();
      yTrajectory = new MinimumJerkTrajectory();
      zTrajectory = new ParabolicTrajectory();
      initialized = false;
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, TimeInterval timeInterval)
   {
      initializeTrajectory(initialPosition, finalPosition, groundClearance, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, double duration)
   {
      initializeTrajectory(initialPosition, finalPosition, groundClearance, 0, duration);
   }

   public void initializeTrajectory(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, double startTime, double endTime)
   {
      super.initializeAdjustmentTrajectory(initialPosition, finalPosition, startTime, endTime);

      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());
      xTrajectory.setMoveParameters(initialPosition.getX(), 0, 0, finalPosition.getX(), 0, 0, timeInterval.getDuration());
      yTrajectory.setMoveParameters(initialPosition.getY(), 0, 0, finalPosition.getY(), 0, 0, timeInterval.getDuration());
      zTrajectory.setMoveParameters(initialPosition.getZ(), midwayPositionZ, finalPosition.getZ(), timeInterval.getDuration());

      initialized = true;
      computeTrajectory(0);
   }

   @Override
   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("parameters must be initialized before computing trajectory");

      xTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      yTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());
      zTrajectory.computeTrajectory(currentTime - timeInterval.getStartTime());

      computeAdjustment(currentTime);

      position.setX(xTrajectory.getPosition() + xTrajectoryAdjustment.getPosition());
      position.setY(yTrajectory.getPosition() + yTrajectoryAdjustment.getPosition());
      position.setZ(zTrajectory.getPosition() + zTrajectoryAdjustment.getPosition());

      velocity.setX(xTrajectory.getVelocity() + xTrajectoryAdjustment.getVelocity());
      velocity.setY(yTrajectory.getVelocity() + yTrajectoryAdjustment.getVelocity());
      velocity.setZ(zTrajectory.getVelocity() + zTrajectoryAdjustment.getVelocity());

      acceleration.setX(xTrajectory.getAcceleration() + xTrajectoryAdjustment.getAcceleration());
      acceleration.setY(yTrajectory.getAcceleration() + yTrajectoryAdjustment.getAcceleration());
      acceleration.setZ(zTrajectory.getAcceleration() + zTrajectoryAdjustment.getAcceleration());
   }
}