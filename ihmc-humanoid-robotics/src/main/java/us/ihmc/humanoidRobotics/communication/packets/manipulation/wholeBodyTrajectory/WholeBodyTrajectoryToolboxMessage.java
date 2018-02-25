package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.TempPreallocatedList;

public class WholeBodyTrajectoryToolboxMessage extends Packet<WholeBodyTrajectoryToolboxMessage>
{
   public WholeBodyTrajectoryToolboxConfigurationMessage configuration;
   public TempPreallocatedList<WaypointBasedTrajectoryMessage> endEffectorTrajectories = new TempPreallocatedList<>(WaypointBasedTrajectoryMessage.class, WaypointBasedTrajectoryMessage::new, 10);
   public TempPreallocatedList<RigidBodyExplorationConfigurationMessage> explorationConfigurations = new TempPreallocatedList<>(RigidBodyExplorationConfigurationMessage.class, RigidBodyExplorationConfigurationMessage::new, 10);
   public TempPreallocatedList<ReachingManifoldMessage> reachingManifolds = new TempPreallocatedList<>(ReachingManifoldMessage.class, ReachingManifoldMessage::new, 10);

   public WholeBodyTrajectoryToolboxMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(WholeBodyTrajectoryToolboxMessage other)
   {
      configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.set(other.configuration);
      MessageTools.copyData(other.endEffectorTrajectories, endEffectorTrajectories);
      MessageTools.copyData(other.explorationConfigurations, explorationConfigurations);
      MessageTools.copyData(other.reachingManifolds, reachingManifolds);
      setPacketInformation(other);
   }

   public void setConfiguration(WholeBodyTrajectoryToolboxConfigurationMessage configuration)
   {
      this.configuration = configuration;
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage getConfiguration()
   {
      return configuration;
   }

   public TempPreallocatedList<WaypointBasedTrajectoryMessage> getEndEffectorTrajectories()
   {
      return endEffectorTrajectories;
   }

   public TempPreallocatedList<RigidBodyExplorationConfigurationMessage> getExplorationConfigurations()
   {
      return explorationConfigurations;
   }

   public TempPreallocatedList<ReachingManifoldMessage> getReachingManifolds()
   {
      return reachingManifolds;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxMessage other, double epsilon)
   {
      if (configuration == null ^ other.configuration == null)
         return false;
      if (configuration != null && !configuration.epsilonEquals(other.configuration, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(endEffectorTrajectories, other.endEffectorTrajectories, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(explorationConfigurations, other.explorationConfigurations, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(reachingManifolds, other.reachingManifolds, epsilon))
         return false;
      if (explorationConfigurations.size() != other.explorationConfigurations.size())
         return false;
      return true;
   }
}
