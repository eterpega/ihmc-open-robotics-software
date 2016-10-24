package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;

/**
 * A geometry represented by a triangle mesh.
 */
@Immutable
public abstract class TriangleGeometryDescription extends GeometryDescription
{
   public abstract TriangleMeshDescription getTriangleMesh();

   @Override public final TriangleGeometryDescription toTriangleGeometry()
   {
      return this;
   }

   @Default
   @Override public TransformDescription getTransform()
   {
      return TransformDescription.IDENTITY;
   }

   public static ImmutableTriangleGeometryDescription.Builder builder()
   {
      return ImmutableTriangleGeometryDescription.builder();
   }
}
