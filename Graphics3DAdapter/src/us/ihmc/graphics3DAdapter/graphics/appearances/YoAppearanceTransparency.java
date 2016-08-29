package us.ihmc.graphics3DAdapter.graphics.appearances;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.tools.color.Color3f;

public abstract class YoAppearanceTransparency implements AppearanceDefinition
{
   private double transparency = 0.0;

   public final double getTransparency()
   {
      return transparency;
   }
   
   public void setTransparency(double transparency)
   {
      this.transparency = transparency;
   }

   public Color3f getColor()
   {
      throw new NotImplementedException("getColor() is not implemented");
   }
}
