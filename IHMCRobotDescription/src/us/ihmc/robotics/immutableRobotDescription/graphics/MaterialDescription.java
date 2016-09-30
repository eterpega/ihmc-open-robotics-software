package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;

import java.awt.Color;

/**
 * Represents a 3D surface material.
 */
@Immutable public abstract class MaterialDescription
{
   private static final MaterialDescription[] DEFAULT_MATERIALS = new MaterialDescription[2];

   public static MaterialDescription black()
   {
      if (DEFAULT_MATERIALS[0] == null)
         DEFAULT_MATERIALS[0] = builder().color(Color.BLACK).build();
      return DEFAULT_MATERIALS[0];
   }

   public static MaterialDescription white()
   {
      if (DEFAULT_MATERIALS[1] == null)
         DEFAULT_MATERIALS[1] = builder().color(Color.WHITE).build();
      return DEFAULT_MATERIALS[1];
   }

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

   public static ImmutableMaterialDescription.Builder builder()
   {
      return ImmutableMaterialDescription.builder();
   }
}
