package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Lazy;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * Represents a 3D box.
 * Code adjusted from JMonkeyEngine.
 */
@Immutable public abstract class BoxDescription extends GeometryDescription
{
   private static final int[] GEOMETRY_INDICES_DATA = {
         2,  1,  0,  3,  2,  0, // back
         6,  5,  4,  7,  6,  4, // right
         10,  9,  8, 11, 10,  8, // front
         14, 13, 12, 15, 14, 12, // left
         18, 17, 16, 19, 18, 16, // top
         22, 21, 20, 23, 22, 20  // bottom
   };

   private static final float[] GEOMETRY_NORMALS_DATA = {
         0,  0, -1,  0,  0, -1,  0,  0, -1,  0,  0, -1, // back
         1,  0,  0,  1,  0,  0,  1,  0,  0,  1,  0,  0, // right
         0,  0,  1,  0,  0,  1,  0,  0,  1,  0,  0,  1, // front
         -1,  0,  0, -1,  0,  0, -1,  0,  0, -1,  0,  0, // left
         0,  1,  0,  0,  1,  0,  0,  1,  0,  0,  1,  0, // top
         0, -1,  0,  0, -1,  0,  0, -1,  0,  0, -1,  0  // bottom
   };

   private static final float[] GEOMETRY_TEXTURE_DATA = {
         1, 0, 0, 0, 0, 1, 1, 1, // back
         1, 0, 0, 0, 0, 1, 1, 1, // right
         1, 0, 0, 0, 0, 1, 1, 1, // front
         1, 0, 0, 0, 0, 1, 1, 1, // left
         1, 0, 0, 0, 0, 1, 1, 1, // top
         1, 0, 0, 0, 0, 1, 1, 1  // bottom
   };

   private static final TriangleMeshDescription geometryData = TriangleMeshDescription.fromTriangleSoupSanitized(
         TriangleSoupDescription.builder()
               .indexBuffer(GEOMETRY_INDICES_DATA)
               .vertexBuffer(createVertices())
               .normalBuffer(createVectorBuffer(GEOMETRY_NORMALS_DATA))
               .textureCoordinatesBuffer(createGeometryTextures())
               .build());

   public abstract double getWidth();

   public abstract double getHeight();

   public abstract double getDepth();

   @Lazy
   public Vector3f getDimensions() {
      return new Vector3f((float)getWidth(), (float)getHeight(), (float)getDepth());
   }

   private static Vector3f[] createVectorBuffer(float[] buffer) {
      Vector3f[] result = new Vector3f[buffer.length / 3];
      for (int i = 0; i < buffer.length; i+= 3)
      {
         result[i / 3] = new Vector3f(buffer[i], buffer[i + 1], buffer[i + 2]);
      }
      return result;
   }

   private static Vector2f[] createGeometryTextures() {
      Vector2f[] result = new Vector2f[GEOMETRY_TEXTURE_DATA.length / 2];
      for (int i = 0; i < GEOMETRY_TEXTURE_DATA.length; i+= 2)
      {
         result[i / 2] = new Vector2f(GEOMETRY_TEXTURE_DATA[i], GEOMETRY_TEXTURE_DATA[i + 1]);
      }
      return result;
   }

   /**
    * Gets the array or vectors representing the 8 vertices of the box.
    *
    * @return a newly created array of vertex vectors.
    */
   private static Vector3f[] createVertices() {
      Vector3f[] axes = {
            new Vector3f(1, 0, 0),
            new Vector3f(0, 1, 0),
            new Vector3f(0, 0, 1)
      };
      float[][] coeffs = {
            {-1, -1, -1},
            {+1, -1, -1},
            {+1, +1, -1},
            {-1, +1, -1},
            {+1, -1, +1},
            {-1, -1, +1},
            {+1, +1, +1},
            {-1, +1, +1}
      };

      Vector3f[] result = new Vector3f[coeffs.length];
      for (int i = 0; i < coeffs.length; i++)
      {
         result[i] = new Vector3f();
         for (int j = 0; j < 3; j++)
         {
            result[i].scaleAdd(coeffs[i][j], axes[j]);
         }
      }

      return result;
   }

   @Override public TriangleGeometryDescription toTriangleGeometry()
   {
      return TriangleGeometryDescription.builder()
            .triangleMesh(geometryData)
            .transform(TransformDescription.fromScale(getDimensions()).compose(getTransform()))
            .build();
   }

   public static ImmutableBoxDescription.Builder builder()
   {
      return ImmutableBoxDescription.builder();
   }
}
