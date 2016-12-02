package us.ihmc.javaFXToolkit.charts.presentationModel.graphicsLayers;

/**
 * Created by amoucheboeuf on 5/17/16.
 */
public class AreaLimits
{

   public double xmin, xmax, ymin, ymax;


   public double getHeight()
   {
      return  ymax - ymin;
   }

   public double getWidth()
   {
      return  xmax - xmin;
   }

   public AreaLimits(){}

   public AreaLimits(double xmin, double xmax, double ymin, double ymax)
   {
      this.xmin = xmin;
      this.xmax = xmax;
      this.ymin = ymin;
      this.ymax = ymax;

      if (xmin > xmax || ymin > ymax)
         throw new RuntimeException();
   }

   public AreaLimits(AreaLimits obj)
   {
      this.xmin = obj.xmin;
      this.xmax = obj.xmax;
      this.ymin = obj.ymin;
      this.ymax = obj.ymax;

   }


   public synchronized void copyValues(AreaLimits obj)
   {
         this.xmin = obj.xmin;
         this.xmax = obj.xmax;
         this.ymin = obj.ymin;
         this.ymax = obj.ymax;
   }


}
