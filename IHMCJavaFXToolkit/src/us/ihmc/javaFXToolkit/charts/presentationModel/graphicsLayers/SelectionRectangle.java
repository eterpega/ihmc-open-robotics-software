package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;

/**
 * Created by amoucheboeuf on 5/31/16.
 */
public class SelectionRectangle
{
   private final AreaLimits areaLimits;
   private final double widthRatio;
   private final double heightRatio;

   public SelectionRectangle(AreaLimits areaLimits, double widthRatio, double heightRatio )
   {
      this.areaLimits = areaLimits;
      this.widthRatio = widthRatio;
      this.heightRatio = heightRatio;
   }

   public AreaLimits getAreaLimits()
   {
      return areaLimits;
   }

   public double getWidthRatio()
   {
      return widthRatio;
   }

   public double getHeightRatio()
   {
      return heightRatio;
   }
}
