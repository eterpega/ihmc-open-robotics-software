package us.ihmc.communication.packets;

import us.ihmc.idl.TempPreallocatedList;

// FIXME Refactor to hold onto a single vertex buffer.
public class PlanarRegionsListMessage extends Packet<PlanarRegionsListMessage>
{
   public TempPreallocatedList<PlanarRegionMessage> planarRegions = new TempPreallocatedList<>(PlanarRegionMessage.class, PlanarRegionMessage::new, 500);

   public PlanarRegionsListMessage()
   {
   }

   @Override
   public void set(PlanarRegionsListMessage other)
   {
      MessageTools.copyData(other.planarRegions, planarRegions);
      setPacketInformation(other);
   }

   public TempPreallocatedList<PlanarRegionMessage> getPlanarRegions()
   {
      return planarRegions;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionsListMessage other, double epsilon)
   {
      return MessageTools.epsilonEquals(planarRegions, other.planarRegions, epsilon);
   }
}
