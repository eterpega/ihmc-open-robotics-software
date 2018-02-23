package us.ihmc.communication.packets;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ObjectDetectorResultPacket extends Packet<ObjectDetectorResultPacket>
{
   public HeatMapPacket heatMap;
   public BoundingBoxesPacket boundingBoxes;

   public ObjectDetectorResultPacket()
   {

   }

   public ObjectDetectorResultPacket(HeatMapPacket heatMap, BoundingBoxesPacket boundingBoxes)
   {
      this.heatMap = heatMap;
      this.boundingBoxes = boundingBoxes;
   }

   @Override
   public boolean epsilonEquals(ObjectDetectorResultPacket other, double epsilon)
   {
      return this.heatMap.epsilonEquals(other.heatMap, epsilon) && this.boundingBoxes.epsilonEquals(other.boundingBoxes, epsilon);
   }
}
