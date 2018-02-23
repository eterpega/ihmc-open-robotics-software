package us.ihmc.communication.packets;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.commons.MathTools;

public class LidarScanMessage extends Packet<LidarScanMessage>
{
   public long robotTimestamp;
   public Point3D32 lidarPosition;
   public Quaternion32 lidarOrientation;
   public float[] scan;

   public LidarScanMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(LidarScanMessage other)
   {
      robotTimestamp = other.robotTimestamp;
      lidarPosition = new Point3D32(other.lidarPosition);
      lidarOrientation = new Quaternion32(other.lidarOrientation);
      scan = Arrays.copyOf(other.scan, other.scan.length);
      setPacketInformation(other);
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public void setLidarPosition(Point3D32 lidarPosition)
   {
      this.lidarPosition = lidarPosition;
   }

   public void setLidarPosition(Point3D lidarPosition)
   {
      this.lidarPosition = new Point3D32(lidarPosition);
   }

   public void setLidarOrientation(Quaternion32 lidarOrientation)
   {
      this.lidarOrientation = lidarOrientation;
   }

   public void setLidarOrientation(Quaternion lidarOrientation)
   {
      this.lidarOrientation = new Quaternion32(lidarOrientation);
   }

   public void setScan(float[] scan)
   {
      this.scan = scan;
   }

   public void setScan(Point3D[] scan)
   {
      this.scan = new float[scan.length * 3];

      int index = 0;

      for (Point3D scanPoint : scan)
      {
         this.scan[index++] = (float) scanPoint.getX();
         this.scan[index++] = (float) scanPoint.getY();
         this.scan[index++] = (float) scanPoint.getZ();
      }
   }

   public void setScan(Point3D32[] scan)
   {
      this.scan = new float[scan.length * 3];

      int index = 0;

      for (Point3D32 scanPoint : scan)
      {
         this.scan[index++] = scanPoint.getX32();
         this.scan[index++] = scanPoint.getY32();
         this.scan[index++] = scanPoint.getZ32();
      }
   }

   public Point3D32 getLidarPosition()
   {
      return lidarPosition;
   }

   public void getLidarPosition(Point3D positionToPack)
   {
      positionToPack.set(lidarPosition);
   }

   public void getLidarPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition);
   }
   
   public Quaternion32 getLidarOrientation()
   {
      return lidarOrientation;
   }

   public void getLidarOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(lidarOrientation);
   }

   public void getLidarOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarOrientation);
   }

   public void getLidarPose(FramePose3D poseToPack)
   {
      poseToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), lidarPosition, lidarOrientation);
   }

   public int getNumberOfScanPoints()
   {
      return scan.length / 3;
   }

   public Point3D32 getScanPoint3f(int index)
   {
      Point3D32 scanPoint = new Point3D32();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, Point3D32 scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public Point3D getScanPoint3d(int index)
   {
      Point3D scanPoint = new Point3D();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, Point3D scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public FramePoint3D getScanFramePoint(int index)
   {
      FramePoint3D scanPoint = new FramePoint3D();
      getScanPoint(index, scanPoint);
      return scanPoint;
   }

   public void getScanPoint(int index, FramePoint3D scanPointToPack)
   {
      index *= 3;
      scanPointToPack.setToZero(ReferenceFrame.getWorldFrame());
      scanPointToPack.setX(scan[index++]);
      scanPointToPack.setY(scan[index++]);
      scanPointToPack.setZ(scan[index++]);
   }

   public Point3D32[] getScanPoint3fs()
   {
      Point3D32[] scanPoints = new Point3D32[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3f(index);
      return scanPoints;
   }

   public Point3D[] getScanPoint3ds()
   {
      Point3D[] scanPoints = new Point3D[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanPoint3d(index);
      return scanPoints;
   }

   public FramePoint3D[] getScanFramePoints()
   {
      FramePoint3D[] scanPoints = new FramePoint3D[getNumberOfScanPoints()];
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPoints[index] = getScanFramePoint(index);
      return scanPoints;
   }

   public void getScanPoint3fs(List<Point3D32> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3f(index));
   }

   public void getScanPoint3ds(List<Point3D> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanPoint3d(index));
   }

   public void getScanFramePoints(List<FramePoint3D> scanPointsToPack)
   {
      scanPointsToPack.clear();
      for (int index = 0; index < getNumberOfScanPoints(); index++)
         scanPointsToPack.add(getScanFramePoint(index));
   }

   @Override
   public boolean epsilonEquals(LidarScanMessage other, double epsilon)
   {
      if (!lidarPosition.epsilonEquals(other.lidarPosition, (float) epsilon))
         return false;
      if (!lidarOrientation.epsilonEquals(other.lidarOrientation, (float) epsilon))
         return false;
      if (scan.length != other.scan.length)
         return false;
      for (int i = 0; i < scan.length; i++)
      {
         if (!MathTools.epsilonEquals(scan[i], other.scan[i], epsilon))
            return false;
      }
      return true;
   }
}
