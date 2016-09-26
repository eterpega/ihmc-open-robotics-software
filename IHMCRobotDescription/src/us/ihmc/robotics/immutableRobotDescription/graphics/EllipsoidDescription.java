package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;

import javax.vecmath.Vector3d;

/**
 * A 3D ellipsoid representation.
 */
@Immutable public abstract class EllipsoidDescription extends GeometryDescription
{

   public abstract double getRadiusX();

   public abstract double getRadiusY();

   public abstract double getRadiusZ();

   public Vector3d getRadii() {
      return new Vector3d(getRadiusX(), getRadiusY(), getRadiusZ());
   }

   @Override public TriangleGeometryDescription toTriangleGeometry()
   {
      // reuse immutable geometry data and just scale it based on radius
      return new TriangleGeometryDescriptionBuilder()
            .triangleMesh(SphereDescription.GEOMETRY_DATA)
            .transform(TransformDescription.fromScale(getRadii()).compose(getTransform()))
            .build();
   }
}
