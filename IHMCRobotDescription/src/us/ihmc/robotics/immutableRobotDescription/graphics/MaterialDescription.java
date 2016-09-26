package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;

import java.awt.Color;

/**
 * Represents a 3D surface material.
 */
@Immutable public abstract class MaterialDescription
{
   public static final MaterialDescription BLACK = new MaterialDescriptionBuilder().color(Color.BLACK).build();
   public static final MaterialDescription WHITE = new MaterialDescriptionBuilder().color(Color.WHITE).build();

   /**
    * Diffuse color.
    * @return color
    */
   public abstract Color getColor();

   /**
    * The "shiny" color of this material.
    * @return reflectance color
    */
   public Color getReflectanceColor()
   {
      return Color.WHITE;
   }

   /**
    * Roughness 0 means a glossy object, roughness 1 a fully diffuse one.
    * @return roughness
    */
   public double getRoughness()
   {
      return 1;
   }
}
