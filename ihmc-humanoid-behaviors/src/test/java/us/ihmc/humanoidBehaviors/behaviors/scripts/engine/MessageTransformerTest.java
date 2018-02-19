package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import static org.junit.Assert.assertTrue;

import java.util.Random;
import java.util.concurrent.TimeUnit;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.DisableOnDebug;
import org.junit.rules.Timeout;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.RandomHumanoidMessages;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.driving.VehiclePosePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.idl.PreallocatedList;

public class MessageTransformerTest
{
   @Rule
   public DisableOnDebug disableOnDebug = new DisableOnDebug(new Timeout(30, TimeUnit.SECONDS));

   @Test(timeout = Integer.MAX_VALUE)
   public void testHandTrajectoryMessage()
   {
      Random random = new Random(6543);

      HandTrajectoryMessage original = RandomHumanoidMessages.nextHandTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      HandTrajectoryMessage expected = new HandTrajectoryMessage(original);
      for (int i = 0; i < expected.se3Trajectory.taskspaceTrajectoryPoints.size(); i++)
      {
         SE3TrajectoryPointMessage trajectoryPoint = expected.se3Trajectory.taskspaceTrajectoryPoints.get(i);
         trajectoryPoint.position.applyTransform(transform);
         trajectoryPoint.orientation.applyTransform(transform);
         trajectoryPoint.linearVelocity.applyTransform(transform);
         trajectoryPoint.angularVelocity.applyTransform(transform);
      }

      HandTrajectoryMessage actual = new HandTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testPelvisHeightTrajectoryMessage()
   {
      Random random = new Random(6543);

      PelvisHeightTrajectoryMessage original = RandomHumanoidMessages.nextPelvisHeightTrajectoryMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      PelvisHeightTrajectoryMessage expected = new PelvisHeightTrajectoryMessage(original);
      for (int i = 0; i < expected.euclideanTrajectory.taskspaceTrajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage trajectoryPoint = expected.euclideanTrajectory.taskspaceTrajectoryPoints.get(i);
         trajectoryPoint.position.applyTransform(transform);
         trajectoryPoint.linearVelocity.applyTransform(transform);
      }

      PelvisHeightTrajectoryMessage actual = new PelvisHeightTrajectoryMessage(original);

      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testAdjustFootstepMessage()
   {
      Random random = new Random(6543);

      AdjustFootstepMessage original = RandomHumanoidMessages.nextAdjustFootstepMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      AdjustFootstepMessage expected = new AdjustFootstepMessage(original);
      expected.location.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      AdjustFootstepMessage actual = new AdjustFootstepMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testFootstepDataMessage()
   {
      Random random = new Random(6543);

      FootstepDataMessage original = RandomHumanoidMessages.nextFootstepDataMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      FootstepDataMessage expected = new FootstepDataMessage(original);
      expected.location.applyTransform(transform);
      expected.orientation.applyTransform(transform);
      for (Point3D waypoint : expected.positionWaypoints.toArray())
         waypoint.applyTransform(transform);

      FootstepDataMessage actual = new FootstepDataMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testFootstepDataListMessage()
   {
      Random random = new Random(6543);

      FootstepDataListMessage original = RandomHumanoidMessages.nextFootstepDataListMessage(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      FootstepDataListMessage expected = new FootstepDataListMessage(original);
      PreallocatedList<FootstepDataMessage> footstepDataList = expected.footstepDataList;
      for (int i = 0; i < footstepDataList.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
         footstepDataMessage.location.applyTransform(transform);
         footstepDataMessage.orientation.applyTransform(transform);
         for (Point3D waypoint : footstepDataMessage.positionWaypoints.toArray())
            waypoint.applyTransform(transform);
      }

      FootstepDataListMessage actual = new FootstepDataListMessage(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testVehiclePosePacket()
   {
      Random random = new Random(6543);

      VehiclePosePacket original = RandomHumanoidMessages.nextVehiclePosePacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      VehiclePosePacket expected = new VehiclePosePacket(original);
      expected.position.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      VehiclePosePacket actual = new VehiclePosePacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }

   @Test(timeout = Integer.MAX_VALUE)
   public void testVideoPacket()
   {
      Random random = new Random(6543);

      VideoPacket original = RandomHumanoidMessages.nextVideoPacket(random);

      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      VideoPacket expected = new VideoPacket(original);
      expected.position.applyTransform(transform);
      expected.orientation.applyTransform(transform);

      VideoPacket actual = new VideoPacket(original);
      MessageTransformer.transform(actual, transform);

      assertTrue(expected.epsilonEquals(actual, 1.0e-5));
   }
}
