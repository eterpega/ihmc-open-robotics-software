package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionsListMessage extends Packet<PlanarRegionsListMessage>
{
   public List<PlanarRegionMessage> planarRegions;

   public PlanarRegionsListMessage()
   {
   }

   public PlanarRegionsListMessage(List<PlanarRegionMessage> planarRegions)
   {
      this.planarRegions = planarRegions;
   }

   @Override
   public void set(PlanarRegionsListMessage other)
   {
      planarRegions = new ArrayList<>();
      for (PlanarRegionMessage region : other.planarRegions)
         planarRegions.add(region);
      setPacketInformation(other);
   }

   public List<PlanarRegionMessage> getPlanarRegions()
   {
      return planarRegions;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionsListMessage other, double epsilon)
   {
      if (planarRegions.size() != other.planarRegions.size())
         return false;
      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegionMessage thisPlanarRegion = planarRegions.get(i);
         PlanarRegionMessage otherPlanarRegion = other.planarRegions.get(i);
         if (!thisPlanarRegion.epsilonEquals(otherPlanarRegion, epsilon))
            return false;
      }
      return true;
   }
}
