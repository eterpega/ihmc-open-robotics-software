package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Default;

/**
 * Represents a geometric shape and its transforms in the 3D space.
 */
public abstract class GeometryDescription implements Transformable
{
   public static final GeometryDescription EMPTY = new GeometryDescription()
   {
      @Override public TriangleGeometryDescription toTriangleGeometry()
      {
         return TriangleGeometryDescription.builder().triangleMesh(TriangleMeshDescription.empty()).build();
      }
   };

   public @Default MaterialDescription getMaterial() {
      return MaterialDescription.white();
   }

   public @Override @Default TransformDescription getTransform() {
      return TransformDescription.IDENTITY;
   }

   public abstract TriangleGeometryDescription toTriangleGeometry();
}
