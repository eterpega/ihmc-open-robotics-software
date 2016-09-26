package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * Represents a 3D Sphere.
 * The code was taken and adjusted from JMonkeyEngine.
 */
@Immutable public abstract class SphereDescription extends GeometryDescription
{
   final static TriangleMeshDescription GEOMETRY_DATA = createGeometryData(1, 16, 16);

   public abstract double getRadius();

   /**
    * creates the vertices based on the radius, radial and zSamples.
    */
   private static TriangleMeshDescription createGeometryData(float radius, int zSamples, int radialSamples) {
      // allocate vertices
      int vertCount = (zSamples - 2) * (radialSamples + 1) + 2;

      Vector3f[] posBuf = new Vector3f[vertCount];

      // allocate normals if requested
      Vector3f[] normBuf = new Vector3f[vertCount];

      // allocate texture coordinates
      Vector2f[] texBuf = new Vector2f[vertCount];

      int[] indexBuf = createIndexData(vertCount, zSamples, radialSamples);

      // generate geometry
      float fInvRS = 1.0f / radialSamples;
      float fZFactor = 2.0f / (zSamples - 1);

      // Generate points on the unit circle to be used in computing the mesh
      // points on a sphere slice.
      float[] afSin = new float[(radialSamples + 1)];
      float[] afCos = new float[(radialSamples + 1)];
      for (int iR = 0; iR < radialSamples; iR++) {
         float fAngle = (float)Math.PI  *2 * fInvRS * iR;
         afCos[iR] = (float)Math.cos(fAngle);
         afSin[iR] = (float)Math.sin(fAngle);
      }
      afSin[radialSamples] = afSin[0];
      afCos[radialSamples] = afCos[0];

      // generate the sphere itself
      int i = 0;
      for (int iZ = 1; iZ < (zSamples - 1); iZ++) {
         float fAFraction = 0.5f * (float)Math.PI * (-1.0f + fZFactor * iZ); // in (-pi/2, pi/2)
         float fZFraction = (float)Math.sin(fAFraction); // in (-1,1)
         float fZ = radius * fZFraction;

         // compute center of slice
         Vector3f kSliceCenter = new Vector3f();
         kSliceCenter.z += fZ;

         // compute radius of slice
         float fSliceRadius = (float)Math.sqrt(Math.abs(radius * radius
                                                               - fZ * fZ));

         // compute slice vertices with duplication at end point
         for (int iR = 0; iR < radialSamples; iR++) {
            float fRadialFraction = iR * fInvRS; // in [0,1)
            Vector3f kRadial = new Vector3f(afCos[iR], afSin[iR], 0);
            Vector3f offset = new Vector3f();
            offset.scale(fSliceRadius, kRadial);
            Vector3f pos = new Vector3f(kSliceCenter);
            pos.add(offset);
            posBuf[i] = pos;

            {
               Vector3f kNormal = new Vector3f(offset);
               kNormal.normalize();
               normBuf[i] = kNormal;
            }

            texBuf[i] = new Vector2f(fRadialFraction, 0.5f * (fZFraction + 1.0f));

            i++;
         }

         texBuf[i] = new Vector2f(1.0f, 0.5f * (fZFraction + 1.0f));

         i++;
      }

      // south pole
      posBuf[i] = new Vector3f(0, 0, -radius);
      normBuf[i] = new Vector3f(0, 0, -1);
      texBuf[i] = new Vector2f(0, 0);

      i++;

      // north pole
      posBuf[i] = new Vector3f(0, 0, radius);
      normBuf[i] = new Vector3f(0, 0, 1);
      texBuf[i] = new Vector2f(0.5f, 1.0f);

      return TriangleMeshDescription.fromTriangleSoupSanitized(new TriangleSoupDescriptionBuilder()
            .indexBuffer(indexBuf)
            .vertexBuffer(posBuf)
            .normalBuffer(normBuf)
            .textureCoordinatesBuffer(texBuf)
            .build());
   }

   /**
    * creates the indices for rendering the sphere.
    */
   private static int[] createIndexData(int vertCount, int zSamples, int radialSamples) {
      // allocate connectivity
      int triCount = 2 * (zSamples - 2) * radialSamples;
      int[] idxBuf = new int[3 * triCount];
      int bufIndex = 0;

      // generate connectivity
      int index = 0;
      for (int iZ = 0, iZStart = 0; iZ < (zSamples - 3); iZ++) {
         int i0 = iZStart;
         int i1 = i0 + 1;
         iZStart += (radialSamples + 1);
         int i2 = iZStart;
         int i3 = i2 + 1;
         for (int i = 0; i < radialSamples; i++, index += 6) {
            idxBuf[bufIndex++] = i0++;
            idxBuf[bufIndex++] = i1;
            idxBuf[bufIndex++] = i2;
            idxBuf[bufIndex++] = i1++;
            idxBuf[bufIndex++] = i3++;
            idxBuf[bufIndex++] = i2++;
         }
      }

      // south pole triangles
      for (int i = 0; i < radialSamples; i++, index += 3) {
         idxBuf[bufIndex++] = i;
         idxBuf[bufIndex++] = (vertCount - 2);
         idxBuf[bufIndex++] = (i + 1);
      }

      // north pole triangles
      int iOffset = (zSamples - 3) * (radialSamples + 1);
      for (int i = 0; i < radialSamples; i++, index += 3) {
         idxBuf[bufIndex++] = (i + iOffset);
         idxBuf[bufIndex++] = (i + 1 + iOffset);
         idxBuf[bufIndex++] = (vertCount - 1);
      }

      return idxBuf;
   }

   @Override public TriangleGeometryDescription toTriangleGeometry()
   {
      // reuse immutable geometry data and just scale it based on radius
      return new TriangleGeometryDescriptionBuilder()
            .triangleMesh(GEOMETRY_DATA)
            .transform(TransformDescription.fromScale((float)getRadius()).compose(getTransform()))
            .build();
   }
}
