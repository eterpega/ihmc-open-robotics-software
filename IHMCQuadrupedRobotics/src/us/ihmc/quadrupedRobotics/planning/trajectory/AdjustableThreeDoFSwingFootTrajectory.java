package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public abstract class AdjustableThreeDoFSwingFootTrajectory
{
   protected final MinimumJerkTrajectory xTrajectoryAdjustment;
   protected final MinimumJerkTrajectory yTrajectoryAdjustment;
   protected final MinimumJerkTrajectory zTrajectoryAdjustment;
   protected final FramePoint initialPosition;
   protected final FramePoint finalPosition;
   protected final FramePoint position;
   protected final FrameVector velocity;
   protected final FrameVector acceleration;
   protected ReferenceFrame referenceFrame;
   protected TimeInterval timeInterval;
   protected TimeInterval timeIntervalOfAdjustment;

   public AdjustableThreeDoFSwingFootTrajectory()
   {
      timeIntervalOfAdjustment = new TimeInterval();
      zTrajectoryAdjustment = new MinimumJerkTrajectory();
      xTrajectoryAdjustment = new MinimumJerkTrajectory();
      finalPosition = new FramePoint();
      position = new FramePoint();
      timeInterval = new TimeInterval();
      acceleration = new FrameVector();
      initialPosition = new FramePoint();
      yTrajectoryAdjustment = new MinimumJerkTrajectory();
      referenceFrame = ReferenceFrame.getWorldFrame();
      velocity = new FrameVector();
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

   protected void initializeAdjustmentTrajectory(FramePoint initialPosition, FramePoint finalPosition, double startTime, double endTime)
   {
      finalPosition.checkReferenceFrameMatch(initialPosition);
      this.initialPosition.setIncludingFrame(initialPosition);
      this.finalPosition.setIncludingFrame(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();
      timeInterval.setInterval(startTime, endTime);
      timeIntervalOfAdjustment.setInterval(startTime, endTime);

      xTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      yTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());
      zTrajectoryAdjustment.setMoveParameters(0, 0, 0, 0, 0, 0, timeInterval.getDuration());

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
   }

   public void adjustTrajectory(FramePoint finalPosition, double currentTime)
   {
      computeTrajectory(currentTime);
      timeIntervalOfAdjustment.setStartTime(Math.min(Math.max(currentTime, timeInterval.getStartTime()), timeInterval.getEndTime()));
      xTrajectoryAdjustment.setMoveParameters(xTrajectoryAdjustment.getPosition(), xTrajectoryAdjustment.getVelocity(), xTrajectoryAdjustment.getAcceleration(), finalPosition.getX() - this.finalPosition.getX(), (finalPosition.getX() - this.finalPosition.getX()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      yTrajectoryAdjustment.setMoveParameters(yTrajectoryAdjustment.getPosition(), yTrajectoryAdjustment.getVelocity(), yTrajectoryAdjustment.getAcceleration(), finalPosition.getY() - this.finalPosition.getY(), (finalPosition.getY() - this.finalPosition.getY()) / timeInterval.getDuration(), 0, timeIntervalOfAdjustment.getDuration());
      zTrajectoryAdjustment.setMoveParameters(zTrajectoryAdjustment.getPosition(), zTrajectoryAdjustment.getVelocity(), zTrajectoryAdjustment.getAcceleration(), finalPosition.getZ() - this.finalPosition.getZ(), 0, 0, timeIntervalOfAdjustment.getDuration());
   }

   public void computeAdjustment(double currentTime)
   {
      xTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      yTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
      zTrajectoryAdjustment.computeTrajectory(currentTime - timeIntervalOfAdjustment.getStartTime());
   }

   public abstract void computeTrajectory(double currentTime);
}
