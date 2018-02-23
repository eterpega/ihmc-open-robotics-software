package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * User: Matt Date: 1/18/13
 */
@RosMessagePacket(documentation = "This message gives the status of the current footstep from the controller as well as the position\n"
      + "and orientation of the footstep in world cooredinates. ", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/output/footstep_status")
public class FootstepStatus extends SettablePacket<FootstepStatus>
{
   public enum Status
   {
      @RosEnumValueDocumentation(documentation = "execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state")
      STARTED, @RosEnumValueDocumentation(documentation = "a footstep is completed")
      COMPLETED
   }

   @RosExportedField(documentation = "The current footstep status enum value.")
   public Status status;
   @RosExportedField(documentation = "footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\n"
         + "FootstepDataListMessage.")
   public int footstepIndex;

   @RosExportedField(documentation = "The robot side (left or right) that this footstep status correlates to.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "desiredFootPositionInWorld gives the position of the desired position sent to the controller as opposed\n"
         + "to where the foot actually landed")
   public Point3D desiredFootPositionInWorld;
   @RosExportedField(documentation = "desiredFootOrientationInWorld gives the desired orientation of the foot sent to the controller as opposed to\n"
         + "the orientation where the foot actually is")
   public Quaternion desiredFootOrientationInWorld;
   @RosExportedField(documentation = "actualFootPositionInWorld gives the position of where the foot actually landed as opposed\n"
         + "to the desired position sent to the controller")
   public Point3D actualFootPositionInWorld;
   @RosExportedField(documentation = "actualFootOrientationInWorld gives the orientation the foot is actually in as opposed to\n"
         + "the desired orientation sent to the controller")
   public Quaternion actualFootOrientationInWorld;

   public FootstepStatus()
   {
   }

   public FootstepStatus(Status status, int footstepIndex)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = null;
      this.actualFootOrientationInWorld = null;
      this.robotSide = null;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3D actualFootPositionInWorld, Quaternion actualFootOrientationInWorld)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;

      this.robotSide = null;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3D actualFootPositionInWorld, Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = null;
      this.desiredFootOrientationInWorld = null;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      this.robotSide = robotSide;
   }

   public FootstepStatus(Status status, int footstepIndex, Point3D desiredFootPositionInWorld, Quaternion desiredFootOrientationInWorld,
                         Point3D actualFootPositionInWorld, Quaternion actualFootOrientationInWorld, RobotSide robotSide)
   {
      this.status = status;
      this.footstepIndex = footstepIndex;
      this.desiredFootPositionInWorld = desiredFootPositionInWorld;
      this.desiredFootOrientationInWorld = desiredFootOrientationInWorld;
      this.actualFootPositionInWorld = actualFootPositionInWorld;
      this.actualFootOrientationInWorld = actualFootOrientationInWorld;
      this.robotSide = robotSide;
   }

   @Override
   public void set(FootstepStatus other)
   {
      status = other.status;
      footstepIndex = other.footstepIndex;
      robotSide = other.robotSide;

      if (desiredFootPositionInWorld == null)
         desiredFootPositionInWorld = new Point3D();
      if (desiredFootOrientationInWorld == null)
         desiredFootOrientationInWorld = new Quaternion();

      if (other.desiredFootPositionInWorld == null)
         desiredFootPositionInWorld.setToNaN();
      else
         desiredFootPositionInWorld.set(other.desiredFootPositionInWorld);

      if (other.desiredFootOrientationInWorld == null)
         desiredFootOrientationInWorld.setToNaN();
      else
         desiredFootOrientationInWorld.set(other.desiredFootOrientationInWorld);

      if (actualFootPositionInWorld == null)
         actualFootPositionInWorld = new Point3D();
      if (actualFootOrientationInWorld == null)
         actualFootOrientationInWorld = new Quaternion();

      if (other.actualFootPositionInWorld == null)
         actualFootPositionInWorld.setToNaN();
      else
         actualFootPositionInWorld.set(other.actualFootPositionInWorld);

      if (other.actualFootOrientationInWorld == null)
         actualFootOrientationInWorld.setToNaN();
      else
         actualFootOrientationInWorld.set(other.actualFootOrientationInWorld);
   }

   public Status getStatus()
   {
      return status;
   }

   public int getFootstepIndex()
   {
      return footstepIndex;
   }

   @Override
   public String toString()
   {
      return "FootstepStatus{" + status + ", index: " + footstepIndex + "}";
   }

   public Point3D getDesiredFootPositionInWorld()
   {
      if (desiredFootPositionInWorld != null)
         return desiredFootPositionInWorld;
      return null;
   }

   public Quaternion getDesiredFootOrientationInWorld()
   {
      if (desiredFootOrientationInWorld != null)
         return desiredFootOrientationInWorld;
      return null;
   }

   public Point3D getActualFootPositionInWorld()
   {
      if (actualFootPositionInWorld != null)
         return actualFootPositionInWorld;
      return null;
   }

   public Quaternion getActualFootOrientationInWorld()
   {
      if (actualFootOrientationInWorld != null)
         return actualFootOrientationInWorld;
      return null;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setStatus(Status status)
   {
      this.status = status;
   }

   public void setFootstepIndex(int footstepIndex)
   {
      this.footstepIndex = footstepIndex;
   }

   public void setActualFootOrientationInWorld(Quaternion actualFootOrientationInWorld)
   {
      if (this.actualFootOrientationInWorld == null)
      {
         this.actualFootOrientationInWorld = new Quaternion();
      }
      this.actualFootOrientationInWorld.set(actualFootOrientationInWorld);
   }

   public void setActualFootPositionInWorld(Point3D actualFootPositionInWorld)
   {
      if (this.actualFootPositionInWorld == null)
      {
         this.actualFootPositionInWorld = new Point3D();
      }
      this.actualFootPositionInWorld.set(actualFootPositionInWorld);
   }

   public void setDesiredFootOrientationInWorld(Quaternion desiredFootOrientationInWorld)
   {
      if (this.desiredFootOrientationInWorld == null)
      {
         this.desiredFootOrientationInWorld = new Quaternion();
      }
      this.desiredFootOrientationInWorld.set(desiredFootOrientationInWorld);
   }

   public void setDesiredFootPositionInWorld(Point3D desiredFootPositionInWorld)
   {
      if (this.desiredFootPositionInWorld == null)
      {
         this.desiredFootPositionInWorld = new Point3D();
      }
      this.desiredFootPositionInWorld.set(desiredFootPositionInWorld);
   }

   @Override
   public boolean equals(Object other)
   {
      if (other instanceof FootstepStatus)
      {
         FootstepStatus otherFoostepStatus = (FootstepStatus) other;
         boolean sameStatus = otherFoostepStatus.getStatus() == getStatus();
         boolean sameIndex = otherFoostepStatus.getFootstepIndex() == getFootstepIndex();

         return sameStatus && sameIndex;
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(FootstepStatus other, double epsilon)
   {
      return this.equals(other);
   }
}
