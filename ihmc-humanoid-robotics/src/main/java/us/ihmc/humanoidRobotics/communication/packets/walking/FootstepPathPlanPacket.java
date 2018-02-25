package us.ihmc.humanoidRobotics.communication.packets.walking;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.TempPreallocatedList;

public class FootstepPathPlanPacket extends Packet<FootstepPathPlanPacket>
{

   public boolean goalsValid;
   public FootstepDataMessage start = HumanoidMessageTools.createFootstepDataMessage();
   public TempPreallocatedList<FootstepDataMessage> originalGoals = new TempPreallocatedList<>(FootstepDataMessage.class, HumanoidMessageTools::createFootstepDataMessage, 30);
   public TempPreallocatedList<FootstepDataMessage> pathPlan = new TempPreallocatedList<>(FootstepDataMessage.class, HumanoidMessageTools::createFootstepDataMessage, 30);
   public TIntArrayList footstepUnknown = new TIntArrayList(); // TODO change back to boolean list with moving to DDS
   public double subOptimality;
   public double pathCost = Double.POSITIVE_INFINITY;

   public FootstepPathPlanPacket()
   {
   }

   @Override
   public void set(FootstepPathPlanPacket other)
   {
      goalsValid = other.goalsValid;
      start = HumanoidMessageTools.createFootstepDataMessage();
      start.set(other.start);
      MessageTools.copyData(other.originalGoals, originalGoals);
      MessageTools.copyData(other.pathPlan, pathPlan);
      footstepUnknown.reset();
      footstepUnknown.addAll(footstepUnknown);

      subOptimality = other.subOptimality;
      pathCost = other.pathCost;

      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(FootstepPathPlanPacket other, double epsilon)
   {
      if (goalsValid != other.goalsValid)
         return false;
      if (!start.epsilonEquals(other.start, epsilon))
         return false;
      if (originalGoals.size() != other.originalGoals.size())
         return false;
      if (pathPlan.size() != other.pathPlan.size())
         return false;
      if (footstepUnknown.size() != other.footstepUnknown.size())
         return false;
      if (Math.abs(subOptimality - other.subOptimality) > epsilon)
         return false;
      if (Math.abs(pathCost - other.pathCost) > epsilon)
         return false;
      for (int i = 0; i < originalGoals.size(); i++)
      {
         if (!originalGoals.get(i).epsilonEquals(other.originalGoals.get(i), epsilon))
            return false;
      }
      for (int i = 0; i < pathPlan.size(); i++)
      {
         if (!pathPlan.get(i).epsilonEquals(other.pathPlan.get(i), epsilon))
            return false;
      }
      for (int i = 0; i < footstepUnknown.size(); i++)
      {
         if (footstepUnknown.get(i) != other.footstepUnknown.get(i))
            return false;
      }

      return true;
   }

}
