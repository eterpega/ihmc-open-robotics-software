package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class HatchLocationPacket extends Packet<HatchLocationPacket>
{
   private RigidBodyTransform hatchToWorldTransform;
   private double hatchStepHeight;
   private double hatchOpeningHeight;
   private double hatchWidth;
   private double hatchThickness;
   
   private static final double defaultHatchStepHeight = 0.15;
   private static final double defaultHatchOpeningHeight = 1.60;
   private static final double defaultHatchWidth = 0.86;
   private static final double defaultHatchThickness = 0.10;

   public HatchLocationPacket()
   {
      
   }
   
   public HatchLocationPacket(RigidBodyTransform hatchToWorldTransform)
   {
      this(hatchToWorldTransform, defaultHatchStepHeight, defaultHatchOpeningHeight, defaultHatchWidth, defaultHatchThickness);
      PrintTools.info("Applying default hatch parameters");
   }
   
   public HatchLocationPacket(RigidBodyTransform hatchToWorldTransform, double hatchStepHeight, double hatchOpeningHeight, double hatchWidth, double hatchThickness)
   {
      this.hatchToWorldTransform = hatchToWorldTransform;
      this.hatchStepHeight = hatchStepHeight;
      this.hatchOpeningHeight = hatchOpeningHeight;
      this.hatchWidth = hatchWidth;
      this.hatchThickness = hatchThickness;
   }

   public RigidBodyTransform getHatchTransformToWorld()
   {
      return hatchToWorldTransform;
   }
   
   public double getHatchStepHeight()
   {
      return hatchStepHeight;
   }
   
   public double getHatchOpeningHeight()
   {
      return hatchOpeningHeight;
   }
   
   public double getHatchWidth()
   {
      return hatchWidth;
   }
   
   public double getHatchThickness()
   {
      return hatchThickness;
   }
  
   public boolean epsilonEquals(HatchLocationPacket hatchPacket, double epsilon)
   {
      boolean transformEquals = hatchToWorldTransform.epsilonEquals(hatchPacket.getHatchTransformToWorld(), epsilon);

      return transformEquals;
   }
}
