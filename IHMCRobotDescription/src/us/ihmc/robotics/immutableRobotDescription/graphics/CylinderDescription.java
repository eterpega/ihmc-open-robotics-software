package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * Represents a 3D closed cylinder.
 */
@Immutable public abstract class CylinderDescription extends GeometryDescription
{
   private static final TriangleMeshDescription geometryData = createGeometry(16, 16, 1, 1, 1, true);

   public abstract float getRadius();

   public abstract float getHeight();

   /**
    * Creates the cylinder based on a new set of parameters.
    * Code adjusted from JMonkeyEngine.
    *
    * @param axisSamples the number of samples along the axis.
    * @param radialSamples the number of samples around the radial.
    * @param radius the radius of the bottom of the cylinder.
    * @param radius2 the radius of the top of the cylinder.
    * @param height the cylinder's height.
    * @param closed should the cylinder have top and bottom surfaces.
    */
   private static TriangleMeshDescription createGeometry(int axisSamples, int radialSamples, float radius, float radius2, float height, boolean closed) {
      // Vertices
      int vertCount = axisSamples * (radialSamples + 1) + (closed ? 2 : 0);
      int triCount = ((closed ? 2 : 0) + 2 * (axisSamples - 1)) * radialSamples;

      Vector3f[] vertexBuffer = new Vector3f[vertCount];
      Vector3f[] normalBuffer = new Vector3f[vertCount];
      Vector2f[] texCoordBuffer = new Vector2f[vertCount];
      int[] indexBuffer = new int[3 * triCount];

      // generate geometry
      float inverseRadial = 1.0f / radialSamples;
      float inverseAxisLess = 1.0f / (closed ? axisSamples - 3 : axisSamples - 1);
      float inverseAxisLessTexture = 1.0f / (axisSamples - 1);
      float halfHeight = 0.5f * height;

      // Generate points on the unit circle to be used in computing the mesh
      // points on a cylinder slice.
      float[] sin = new float[radialSamples + 1];
      float[] cos = new float[radialSamples + 1];

      for (int radialCount = 0; radialCount < radialSamples; radialCount++) {
         float angle = (float)(2 * Math.PI * inverseRadial * radialCount);
         cos[radialCount] = (float)Math.cos(angle);
         sin[radialCount] = (float)Math.sin(angle);
      }
      sin[radialSamples] = sin[0];
      cos[radialSamples] = cos[0];

      // calculate normals
      Vector3f[] vNormals = null;
      //Vector3f vNormal = new Vector3f(0, 0, 1);

      if ((height != 0.0f) && (radius != radius2)) {
         vNormals = new Vector3f[radialSamples];
         Vector3f vHeight = new Vector3f(0, 0, 1);vHeight.scale(height);
         Vector3f vRadial = new Vector3f();

         for (int radialCount = 0; radialCount < radialSamples; radialCount++) {
            vRadial.set(cos[radialCount], sin[radialCount], 0.0f);
            Vector3f vRadius = new Vector3f(vRadial); vRadius.scale(radius);
            Vector3f vRadius2 = new Vector3f(vRadial); vRadius2.scale(radius2);
            Vector3f vRadiusDiff = new Vector3f(vRadius2); vRadiusDiff.sub(vRadius);
            Vector3f vMantle = new Vector3f(vHeight); vMantle.sub(vRadiusDiff);
            Vector3f vTangent = new Vector3f(); vTangent.cross(vRadial, new Vector3f(0, 0, 1));
            Vector3f vNormal = new Vector3f(); vNormal.cross(vMantle, vTangent); vNormal.normalize();
            vNormals[radialCount] = vNormal;
         }
      }

      // generate the cylinder itself
      Vector3f tempNormal = new Vector3f();
      int vertexBufferIndex = 0, normalBufferIndex = 0, texBufferIndex = 0;
      for (int axisCount = 0, i = 0; axisCount < axisSamples; axisCount++, i++) {
         float axisFraction;
         float axisFractionTexture;
         int topBottom = 0;
         if (!closed) {
            axisFraction = axisCount * inverseAxisLess; // in [0,1]
            axisFractionTexture = axisFraction;
         } else {
            if (axisCount == 0) {
               topBottom = -1; // bottom
               axisFraction = 0;
               axisFractionTexture = inverseAxisLessTexture;
            } else if (axisCount == axisSamples - 1) {
               topBottom = 1; // top
               axisFraction = 1;
               axisFractionTexture = 1 - inverseAxisLessTexture;
            } else {
               axisFraction = (axisCount - 1) * inverseAxisLess;
               axisFractionTexture = axisCount * inverseAxisLessTexture;
            }
         }

         // compute center of slice
         float z = -halfHeight + height * axisFraction;
         Vector3f sliceCenter = new Vector3f(0, 0, z);

         // compute slice vertices with duplication at end point
         for (int radialCount = 0; radialCount < radialSamples; radialCount++, i++) {
            float radialFraction = radialCount * inverseRadial; // in [0,1)
            tempNormal.set(cos[radialCount], sin[radialCount], 0.0f);

            Vector3f vNormal = new Vector3f(0, 0, 1);
            if (vNormals != null) {
               vNormal = vNormals[radialCount];
            } else if (radius == radius2) {
               vNormal = tempNormal;
            }

            if (topBottom == 0) {
               normalBuffer[normalBufferIndex++] = new Vector3f(vNormal);
            } else {
               normalBuffer[normalBufferIndex++] = new Vector3f(0, 0, topBottom);
            }

            tempNormal.scale((radius - radius2) * axisFraction + radius2);
            tempNormal.add(sliceCenter);
            vertexBuffer[vertexBufferIndex++] = new Vector3f(tempNormal);

            texCoordBuffer[texBufferIndex++] = new Vector2f(radialFraction, axisFractionTexture);
         }

         texCoordBuffer[texBufferIndex++] = new Vector2f(1.0f, axisFractionTexture);
      }

      if (closed) {
         vertexBuffer[vertexBufferIndex++] = new Vector3f(0, 0, -halfHeight); // bottom center
         normalBuffer[normalBufferIndex++] = new Vector3f(0, 0, -1);
         texCoordBuffer[texBufferIndex++] = new Vector2f(0.5f, 0);
         vertexBuffer[vertexBufferIndex] = new Vector3f(0, 0, halfHeight); // top center
         normalBuffer[normalBufferIndex] = new Vector3f(0, 0, 1);
         texCoordBuffer[texBufferIndex] = new Vector2f(0.5f, 1);
      }

      int index = 0;
      // Connectivity
      for (int axisCount = 0, axisStart = 0; axisCount < axisSamples - 1; axisCount++) {
         int i0 = axisStart;
         int i1 = i0 + 1;
         axisStart += radialSamples + 1;
         int i2 = axisStart;
         int i3 = i2 + 1;
         for (int i = 0; i < radialSamples; i++) {
            if (closed && axisCount == 0) {
               indexBuffer[index++] = i0++;
               indexBuffer[index++] = vertCount - 2;
               indexBuffer[index++] = i1++;
            } else if (closed && axisCount == axisSamples - 2) {
               indexBuffer[index++] = i2++;
               indexBuffer[index++] = i3++;
               indexBuffer[index++] = vertCount - 1;
            } else {
               indexBuffer[index++] = i0++;
               indexBuffer[index++] = i1;
               indexBuffer[index++] = i2;
               indexBuffer[index++] = i1++;
               indexBuffer[index++] = i3++;
               indexBuffer[index++] = i2++;
            }
         }
      }

      return TriangleMeshDescription.fromTriangleSoupSanitized(TriangleSoupDescription.builder()
                                                            .indexBuffer(indexBuffer)
                                                            .vertexBuffer(vertexBuffer)
                                                            .normalBuffer(normalBuffer)
                                                            .textureCoordinatesBuffer(texCoordBuffer)
                                                            .build());
   }

   @Override public TriangleGeometryDescription toTriangleGeometry()
   {
      // reuse base geometry and just scale it accordingly
      return TriangleGeometryDescription.builder()
            .triangleMesh(geometryData)
            .transform(TransformDescription.fromScale(new Vector3f(getRadius(), getRadius(), getHeight())).compose(getTransform()))
            .build();
   }

   public static ImmutableCylinderDescription.Builder builder()
   {
      return ImmutableCylinderDescription.builder();
   }
}
