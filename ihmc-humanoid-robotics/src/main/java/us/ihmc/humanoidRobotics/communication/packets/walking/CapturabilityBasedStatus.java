package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.TempPreallocatedList;

public class CapturabilityBasedStatus extends Packet<CapturabilityBasedStatus>
{
   public static final int MAXIMUM_NUMBER_OF_VERTICES = 8;

   public Point2D capturePoint = new Point2D();
   public Point2D desiredCapturePoint = new Point2D();

   public Point3D centerOfMass = new Point3D();

   public TempPreallocatedList<Point2D> leftFootSupportPolygon = new TempPreallocatedList<>(Point2D.class, Point2D::new, MAXIMUM_NUMBER_OF_VERTICES);
   public TempPreallocatedList<Point2D> rightFootSupportPolygon = new TempPreallocatedList<>(Point2D.class, Point2D::new, MAXIMUM_NUMBER_OF_VERTICES);

   public CapturabilityBasedStatus()
   {
      // Empty constructor for serialization
   }

   @Override
   public void set(CapturabilityBasedStatus other)
   {
      capturePoint.set(other.capturePoint);
      desiredCapturePoint.set(other.desiredCapturePoint);
      MessageTools.copyData(other.leftFootSupportPolygon, leftFootSupportPolygon);
      MessageTools.copyData(other.rightFootSupportPolygon, rightFootSupportPolygon);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {

      boolean ret = this.capturePoint.epsilonEquals(other.capturePoint, epsilon);
      ret &= this.desiredCapturePoint.epsilonEquals(other.desiredCapturePoint, epsilon);

      ret &= this.centerOfMass.epsilonEquals(other.centerOfMass, epsilon);

      if (!MessageTools.epsilonEquals(leftFootSupportPolygon, other.leftFootSupportPolygon, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(rightFootSupportPolygon, other.rightFootSupportPolygon, epsilon))
         return false;

      return ret;
   }
}
