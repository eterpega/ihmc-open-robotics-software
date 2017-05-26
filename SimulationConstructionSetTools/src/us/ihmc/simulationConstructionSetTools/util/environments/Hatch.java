package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class Hatch
   {
      private RigidBodyTransform hatchToWorldTransform;
      private double hatchStepHeight;
      private double hatchOpeningHeight;
      private double hatchWidth;
      private double hatchThickness;
      
      public Hatch(RigidBodyTransform hatchToWorldTransform, double hatchStepHeight, double hatchOpeningHeight, double hatchWidth, double hatchThickness)
      {
         this.hatchToWorldTransform = hatchToWorldTransform;
         this.hatchStepHeight = hatchStepHeight;
         this.hatchOpeningHeight = hatchOpeningHeight;
         this.hatchWidth = hatchWidth;
         this.hatchThickness = hatchThickness;
      }
      
      public RigidBodyTransform getHatchToWorldTransform()
      {
         return hatchToWorldTransform;
      }
      
      public double getStepHeight()
      {
         return hatchStepHeight;
      }
      
      public double getOpeningHeight()
      {
         return hatchOpeningHeight;
      }
      
      public double getWidth()
      {
         return hatchWidth;
      }
      
      public double getThickness()
      {
         return hatchThickness;
      }
      
      public void applySafteyMargins()
      {
         hatchStepHeight += 0.03; //was 0.02
         hatchOpeningHeight -= 0.05;
         hatchThickness += 0.00; //0.01 was 0.02
      }
      
      public void printHatchDimensions()
      {
         PrintTools.info("Hatch parameters:");
         PrintTools.info("Transform = " + hatchToWorldTransform.toString());
         PrintTools.info("Step   = " + hatchStepHeight);
         PrintTools.info("Height = " + hatchOpeningHeight);
         PrintTools.info("Width  = " + hatchWidth);
         PrintTools.info("Depth  = " + hatchThickness);
      }
   }