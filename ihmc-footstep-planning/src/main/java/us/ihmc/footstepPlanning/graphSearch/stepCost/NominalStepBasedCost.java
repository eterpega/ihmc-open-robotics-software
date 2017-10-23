package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class NominalStepBasedCost implements FootstepCost
{
   private static double costPerStep = 0.15;
   private static double idealFootstepWidth = 0.22;
   private static double idealFootstepLength = 0.3;

   private static double idealStepLengthWeight = 0.2;
   private static double idealStepWidthInwardWeight = 0.8;
   private static double idealStepWidthOutwardWeight = 0.6;
   private static double forwardStepLengthWeight = 0.9 - idealStepLengthWeight;
   private static double backwardStepLengthWeight = 1.1 - idealStepLengthWeight;

   private static double distanceWeight = 0.9;

   private static double yawWeight = 0.1;
   private static double rollWeight = 0.05;
   private static double pitchWeight = 0.02;

   private static double heightChangeUpWeight = 0.5;
   private static double heightChangeDownWeight = 0.75;
   private static double stepUpDeadband = 0.05;
   private static double stepDownDeadband = -0.03;

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode, RigidBodyTransform snapTransform)
   {
      Point2D startPoint = computeMidFootPoint(startNode, idealFootstepWidth);
      Point2D endPoint = computeMidFootPoint(endNode, idealFootstepLength);

      double euclideanDistance = startPoint.distance(endPoint);
      double yaw = AngleTools.computeAngleDifferenceMinusPiToPi(startNode.getYaw(), endNode.getYaw());
      double[] yawPitchRoll = new double[3];
      snapTransform.getRotationYawPitchRoll(yawPitchRoll);

      double heightChangeCost = computeHeightChangeCost(snapTransform);
      double rollCost = computeRollCost(yawPitchRoll);
      double pitchCost = computePitchCost(yawPitchRoll);
      double yawCost = computeYawCost(yaw);

      //double distanceCost = distanceWeight * euclideanDistance;
      double distanceCost = computeStepXYCost(startNode, endNode);

      return distanceCost + heightChangeCost + rollCost + pitchCost + yawCost + costPerStep;
   }


   private double computeHeightChangeCost(RigidBodyTransform snapTransform)
   {
      double heightChange = snapTransform.getTranslationZ();
      double heightChangeCost = 0.0;
      if (heightChange > stepUpDeadband)
         heightChangeCost = (heightChange - stepUpDeadband) * heightChangeUpWeight;
      else if (heightChange < stepDownDeadband)
         heightChangeCost = -(heightChange - stepDownDeadband) * heightChangeDownWeight;

      return heightChangeCost;
   }

   private double computeRollCost(double[] yawPitchRoll)
   {
      return rollWeight * Math.abs(yawPitchRoll[2]);
   }

   private double computePitchCost(double[] yawPitchRoll)
   {
      return pitchWeight * Math.abs(yawPitchRoll[1]);
   }

   // todo make this cost function a little fancier (e.g. increase step width yaw cost as length increase, and decrease yaw cost as width increases)
   private double computeYawCost(double yaw)
   {
      return yawWeight * Math.abs(yaw);
   }

   public static ReferenceFrame getStanceFrame(FootstepNode node)
   {
      FramePose stanceFootPose = new FramePose(ReferenceFrame.getWorldFrame());
      stanceFootPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      stanceFootPose.setX(node.getX());
      stanceFootPose.setY(node.getY());
      return new PoseReferenceFrame("stanceFrame", stanceFootPose);
   }

   public static Point2D computeMidFootPoint(FootstepNode node, double idealStepWidth)
   {
      ReferenceFrame stanceFrame = getStanceFrame(node);

      FramePoint2D midFootPoint = new FramePoint2D(stanceFrame);
      double ySign = node.getRobotSide().negateIfLeftSide(1.0);
      midFootPoint.setY(ySign * idealStepWidth / 2.0);
      midFootPoint.changeFrame(ReferenceFrame.getWorldFrame());

      return midFootPoint.getPoint();
   }

   // todo make this cost function a little fancier (e.g. increase step width cost as the step length increases as well)
   public double computeStepXYCost(FootstepNode startNode, FootstepNode endNode)
   {
      FramePose startPose = new FramePose(ReferenceFrame.getWorldFrame());
      startPose.setYawPitchRoll(startNode.getYaw(), 0.0, 0.0);
      startPose.setX(startNode.getX());
      startPose.setY(startNode.getY());
      ReferenceFrame startReferenceFrame = new PoseReferenceFrame("startFrame", startPose);

      FramePose endPose = new FramePose(ReferenceFrame.getWorldFrame());
      endPose.setYawPitchRoll(endNode.getYaw(), 0.0, 0.0);
      endPose.setX(endNode.getX());
      endPose.setY(endNode.getY());

      endPose.changeFrame(startReferenceFrame);

      double stepLength = endPose.getX();
      double stepWidth = endNode.getRobotSide().negateIfRightSide(endPose.getY());

      double idealStepLengthCost = idealStepLengthWeight * Math.abs(idealFootstepLength - stepLength);
      double stepLengthCost, idealStepWidthCost;
      if (stepLength > 0.0)
         stepLengthCost = forwardStepLengthWeight * stepLength;
      else
         stepLengthCost = -backwardStepLengthWeight * stepLength;

      if (stepWidth > 0.0)
         idealStepWidthCost = idealStepWidthOutwardWeight * stepWidth;
      else
         idealStepWidthCost = -idealStepWidthInwardWeight * stepWidth;

      return idealStepLengthCost + idealStepWidthCost + stepLengthCost;
   }
}
