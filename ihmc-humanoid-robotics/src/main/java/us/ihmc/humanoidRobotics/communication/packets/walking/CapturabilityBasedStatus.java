package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Arrays;

import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class CapturabilityBasedStatus extends SettablePacket<CapturabilityBasedStatus>
{
   public static final int MAXIMUM_NUMBER_OF_VERTICES = 8;

   public Point2D capturePoint = new Point2D();
   public Point2D desiredCapturePoint = new Point2D();

   public Point3D centerOfMass = new Point3D();

   public int leftFootSupportPolygonLength;
   public Point2D[] leftFootSupportPolygonStore = new Point2D[MAXIMUM_NUMBER_OF_VERTICES];
   public int rightFootSupportPolygonLength;
   public Point2D[] rightFootSupportPolygonStore = new Point2D[MAXIMUM_NUMBER_OF_VERTICES];

   public CapturabilityBasedStatus()
   {
      // Empty constructor for serialization
      for (int i = 0; i < MAXIMUM_NUMBER_OF_VERTICES; i++)
      {
         leftFootSupportPolygonStore[i] = new Point2D();
         rightFootSupportPolygonStore[i] = new Point2D();

      }
   }

   @Override
   public void set(CapturabilityBasedStatus other)
   {
      capturePoint.set(other.capturePoint);
      desiredCapturePoint.set(other.desiredCapturePoint);
      leftFootSupportPolygonLength = other.leftFootSupportPolygonLength;
      rightFootSupportPolygonLength = other.rightFootSupportPolygonLength;
      for (int i = 0; i < MAXIMUM_NUMBER_OF_VERTICES; i++)
      {
         leftFootSupportPolygonStore[i].set(other.leftFootSupportPolygonStore[i]);
         rightFootSupportPolygonStore[i].set(other.rightFootSupportPolygonStore[i]);
      }
   }

   public void setSupportPolygon(RobotSide robotSide, FrameConvexPolygon2d footPolygon)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();

      if (numberOfVertices > MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = MAXIMUM_NUMBER_OF_VERTICES;
      }

      if (robotSide == RobotSide.LEFT)
      {
         leftFootSupportPolygonLength = numberOfVertices;
      }
      else
      {
         rightFootSupportPolygonLength = numberOfVertices;
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         if (robotSide == RobotSide.LEFT)
         {
            footPolygon.getVertex(i, leftFootSupportPolygonStore[i]);
         }
         else
         {
            footPolygon.getVertex(i, rightFootSupportPolygonStore[i]);
         }
      }
   }

   public FramePoint2D getCapturePoint()
   {
      return new FramePoint2D(ReferenceFrame.getWorldFrame(), capturePoint);
   }

   public FramePoint2D getDesiredCapturePoint()
   {
      return new FramePoint2D(ReferenceFrame.getWorldFrame(), desiredCapturePoint);
   }

   public FrameConvexPolygon2d getFootSupportPolygon(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && leftFootSupportPolygonLength != 0)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), Arrays.copyOf(leftFootSupportPolygonStore, leftFootSupportPolygonLength));
      else if (rightFootSupportPolygonStore != null)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), Arrays.copyOf(rightFootSupportPolygonStore, rightFootSupportPolygonLength));
      else
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame());
   }

   public boolean isInDoubleSupport()
   {
      return leftFootSupportPolygonLength != 0 & rightFootSupportPolygonLength != 0;
   }

   public boolean isSupportFoot(RobotSide robotside)
   {
      if (robotside == RobotSide.LEFT)
         return leftFootSupportPolygonLength != 0;
      else
         return rightFootSupportPolygonLength != 0;
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {

      boolean ret = this.capturePoint.epsilonEquals(other.capturePoint, epsilon);
      ret &= this.desiredCapturePoint.epsilonEquals(other.desiredCapturePoint, epsilon);

      ret &= this.centerOfMass.epsilonEquals(other.centerOfMass, epsilon);

      ret &= this.leftFootSupportPolygonLength == other.leftFootSupportPolygonLength;
      ret &= this.rightFootSupportPolygonLength == other.rightFootSupportPolygonLength;

      if (leftFootSupportPolygonStore == null || other.leftFootSupportPolygonStore == null)
      {
         ret &= leftFootSupportPolygonStore == null && other.leftFootSupportPolygonStore == null;
      }
      else
      {
         for (int i = 0; i < leftFootSupportPolygonStore.length; i++)
         {
            ret &= this.leftFootSupportPolygonStore[i].epsilonEquals(other.leftFootSupportPolygonStore[i], epsilon);
         }
      }

      if (rightFootSupportPolygonStore == null || other.rightFootSupportPolygonStore == null)
      {
         ret &= rightFootSupportPolygonStore == null && other.rightFootSupportPolygonStore == null;
      }
      else
      {
         for (int i = 0; i < rightFootSupportPolygonStore.length; i++)
         {
            ret &= this.rightFootSupportPolygonStore[i].epsilonEquals(other.rightFootSupportPolygonStore[i], epsilon);
         }
      }

      return ret;
   }
}
