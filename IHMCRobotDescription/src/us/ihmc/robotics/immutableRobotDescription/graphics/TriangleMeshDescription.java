package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.jetbrains.annotations.Nullable;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

import static java.lang.Float.isInfinite;
import static java.lang.Float.isNaN;

/**
 * Represents a cleaned up version of {@link TriangleSoupDescription}. Cannot contain
 * NaN or infinite values in any of the buffers, the index buffer is verified
 * to be in range and all the buffers have appropriate sizes.
 */
public class TriangleMeshDescription implements ShapeDescription
{
   public static final TriangleMeshDescription EMPTY = new TriangleMeshDescription(new TriangleSoupDescriptionBuilder()
                                                                                         .indexBuffer()
                                                                                         .vertexBuffer()
                                                                                         .normalBuffer()
                                                                                         .textureCoordinatesBuffer()
                                                                                         .build(), CullMode.CullNone);

   private final TriangleSoupDescription triangleSoup;
   private final CullMode cullMode;



   /**
    * Allows enabling back-face or front-face culling for the renderer
    */
   public enum CullMode
   {
      CullNone, CullBack, CullFront
   }

   private TriangleMeshDescription(TriangleSoupDescription triangleSoup, CullMode cullMode)
   {
      this.triangleSoup = triangleSoup;
      this.cullMode = cullMode;
   }

   public int[] getIndexBuffer()
   {
      return triangleSoup.getIndexBuffer();
   }

   public Vector3f[] getVertexBuffer()
   {
      return triangleSoup.getVertexBuffer();
   }

   public @Nullable Vector3f[] getNormalBuffer()
   {
      return triangleSoup.getNormalBuffer();
   }

   public @Nullable Vector2f[] getTextureCoordinatesBuffer()
   {
      return triangleSoup.getTextureCoordinatesBuffer();
   }

   public int getFaceCount()
   {
      return getIndexBuffer().length / 3;
   }

   public Vector3f getVertex(int index)
   {
      return getVertexBuffer()[index];
   }

   public Vector3f getNormal(int index, Vector3f defaultNormal)
   {
      return getNormalBuffer() == null ? defaultNormal : getNormalBuffer()[index];
   }

   public Vector2f getTextureCoordinate(int index, Vector2f defaultUV)
   {
      return getTextureCoordinatesBuffer() == null ? defaultUV : getTextureCoordinatesBuffer()[index];
   }

   public int[] getFace(int index)
   {
      index *= 3;
      return new int[] {getIndexBuffer()[index], getIndexBuffer()[index + 1], getIndexBuffer()[index + 2]};
   }

   public CullMode getCullMode()
   {
      return cullMode;
   }

   /**
    * Creates a new triangle mesh from a {@link TriangleSoupDescription}. Fixes
    * any errors in the triangle soup that make the mesh invalid such
    * as wrong triangle vertex order, wrong floating point values etc.
    * @param triangleSoup triangle soup to convert to a mesh
    * @return triangle mesh
    */
   public static TriangleMeshDescription fromTriangleSoupSanitized(TriangleSoupDescription triangleSoup)
   {
      return fromTriangleSoupSanitized(triangleSoup, CullMode.CullNone);
   }

   /**
    * Creates a new triangle mesh from a {@link TriangleSoupDescription}. Fixes
    * any errors in the triangle soup that make the mesh invalid such
    * as wrong triangle vertex order, wrong floating point values etc.
    * @param triangleSoup triangle soup to convert to a mesh
    * @param cullMode triangle cull mode
    * @return triangle mesh
    */
   public static TriangleMeshDescription fromTriangleSoupSanitized(TriangleSoupDescription triangleSoup, CullMode cullMode)
   {
      return new TriangleMeshDescription(sanitizedTriangleSoup(triangleSoup), cullMode);
   }

   /**
    * Creates a new triangle mesh from a {@link TriangleSoupDescription}. This method
    * checks that the triangle soup is a valid mesh and throws an exception
    * when the mesh is invalid. Use {@link #fromTriangleSoupSanitized(TriangleSoupDescription)}
    * to automatically correct any found errors and turn any triangle soup to a triangle
    * mesh.
    * @param triangleSoup triangle soup to convert
    * @return triangle mesh
    * @throws Exception when the given triangle soup is not a valid mesh
    */
   public static TriangleMeshDescription fromTriangleSoup(TriangleSoupDescription triangleSoup) throws Exception
   {
      return fromTriangleSoup(triangleSoup, CullMode.CullNone);
   }

   /**
    * Creates a new triangle mesh from a {@link TriangleSoupDescription}. This method
    * checks that the triangle soup is a valid mesh and throws an exception
    * when the mesh is invalid. Use {@link #fromTriangleSoupSanitized(TriangleSoupDescription)}
    * to automatically correct any found errors and turn any triangle soup to a triangle
    * mesh.
    * @param triangleSoup triangle soup to convert
    * @param cullMode triangle cull mode
    * @return triangle mesh
    * @throws Exception when the given triangle soup is not a valid mesh
    */
   public static TriangleMeshDescription fromTriangleSoup(TriangleSoupDescription triangleSoup, CullMode cullMode) throws Exception
   {
      TriangleSoupDescription sanitized = sanitizedTriangleSoup(triangleSoup);
      if (sanitized.getIndexBuffer() != triangleSoup.getIndexBuffer())
         throw new Exception("Some triangles have wrong orientation");
      if (sanitized.getNormalBuffer() != triangleSoup.getNormalBuffer())
         throw new Exception("Some normals are invalid (Nan, Infinite, zero or not normalized)");
      if (sanitized.getVertexBuffer() != triangleSoup.getVertexBuffer())
         throw new Exception("Some vertices are invalid (Nan or Infinite)");
      if (sanitized.getTextureCoordinatesBuffer() != triangleSoup.getTextureCoordinatesBuffer())
         throw new Exception("Some texture coordinates are invalid (Nan or Infinite");
      return new TriangleMeshDescription(sanitized, cullMode);
   }

   private static TriangleSoupDescription sanitizedTriangleSoup(TriangleSoupDescription triangleSoup) {
      TriangleMeshDescription tempMesh = new TriangleMeshDescription(triangleSoup, CullMode.CullNone);
      FaceCheckResult newFaces = tempMesh.checkFaceOrientationAndNormals();
      Vector3f[] newVertices = tempMesh.checkBuffer(tempMesh.getVertexBuffer(), new Vector3f(), new ValidPredicate<Vector3f>()
      {
         @Override public boolean isValid(Vector3f vertex)
         {
            return TriangleMeshDescription.isValid(vertex);
         }
      });
      Vector2f[] newTexCoords = null;
      if (tempMesh.getTextureCoordinatesBuffer() != null)
      {
         newTexCoords = tempMesh.checkBuffer(tempMesh.getTextureCoordinatesBuffer(), new Vector2f(), new ValidPredicate<Vector2f>()
         {
            @Override public boolean isValid(Vector2f vector)
            {
               return TriangleMeshDescription.isValid(vector);
            }
         });
      }

      return new TriangleSoupDescriptionBuilder().indexBuffer(newFaces.indexBuffer)
                               .vertexBuffer(newVertices)
                               .normalBuffer(newFaces.normalBuffer)
                               .textureCoordinatesBuffer(newTexCoords)
                               .build();
   }

   /**
    * Makes sure that the items in the buffer are all valid. If they are not valid,
    * they are replaced by another valid item from the base buffer. If no valid items exist in the buffer,
    * the default vector is used.
    * @return corrected vertices or {@link #getVertexBuffer()} if the vertex buffer is unchanged
    */
   private <T> T[] checkBuffer(T[] baseBuffer, T defaultVector, ValidPredicate<T> validPredicate)
   {
      T valid = defaultVector;
      for (T vertex : baseBuffer)
      {
         if (validPredicate.isValid(vertex))
         {
            valid = vertex;
            break;
         }
      }

      T[] newBuffer = null;
      for (int i = 0; i < baseBuffer.length; i++)
      {
         if (!validPredicate.isValid(baseBuffer[i]))
         {
            if (newBuffer == null)
               newBuffer = baseBuffer.clone();
            newBuffer[i] = valid;
         }
      }

      return newBuffer != null ? newBuffer : baseBuffer;
   }

   /**
    * Makes sure that the orientation of the face normal corresponds to the order of vertices in the face.
    * @return a new index/normal buffer with fixed orientations or {@link #getIndexBuffer()}/{@link #getNormalBuffer()} if no changes are needed
    */
   private FaceCheckResult checkFaceOrientationAndNormals()
   {
      if (getNormalBuffer() == null)
         return new FaceCheckResult(getIndexBuffer(), getNormalBuffer());

      int[] newIndexBuffer = null;
      Vector3f[] newNormalBuffer = null;
      int faceCount = getFaceCount();
      for (int i = 0; i < faceCount; i++)
      {
         int[] face = getFace(i);
         Vector3f p1 = getVertex(face[0]);
         Vector3f p2 = getVertex(face[1]);
         Vector3f p3 = getVertex(face[2]);
         Vector3f v1 = new Vector3f(p2);
         v1.sub(p1);
         Vector3f v2 = new Vector3f(p3);
         v2.sub(p1);
         Vector3f n1 = getNormal(face[0], null);
         Vector3f n2 = getNormal(face[1], null);
         Vector3f n3 = getNormal(face[2], null);
         Vector3f normal = new Vector3f();
         normal.add(n1);
         normal.add(n2);
         normal.add(n3);

         Vector3f cross = new Vector3f();
         cross.cross(v1, v2);
         float dot = cross.dot(normal);
         if (dot < 0)
         {
            if (newIndexBuffer == null)
               newIndexBuffer = getIndexBuffer().clone();
            newIndexBuffer[i * 3] = getIndexBuffer()[i * 3 + 2];
            newIndexBuffer[i * 3 + 2] = getIndexBuffer()[i * 3];
         }

         if (!isValid(normal))
         {
            if (newNormalBuffer == null)
               newNormalBuffer = getNormalBuffer().clone();

            Vector3f crossVec = new Vector3f(cross);
            if (crossVec.length() == 0)
               crossVec = new Vector3f(0, 1, 0);
            else
               crossVec.normalize();

            if (!isValid(n1))
               newNormalBuffer[face[0]] = crossVec;
            if (!isValid(n2))
               newNormalBuffer[face[1]] = crossVec;
            if (!isValid(n3))
               newNormalBuffer[face[2]] = crossVec;
         }
      }

      return new FaceCheckResult(newIndexBuffer != null ? newIndexBuffer : getIndexBuffer(), newNormalBuffer != null ? newNormalBuffer : getNormalBuffer());
   }

   private static boolean isValid(Vector3f vec)
   {
      return !(isNaN(vec.x) || isNaN(vec.y) || isNaN(vec.z) || isInfinite(vec.x) || isInfinite(vec.y) || isInfinite(vec.z));
   }

   private static boolean isValid(Vector2f vec)
   {
      return !(isNaN(vec.x) || isNaN(vec.y) || isInfinite(vec.x) || isInfinite(vec.y));
   }

   @Override public TriangleMeshDescription toTriangleMesh()
   {
      return this;
   }

   private static class FaceCheckResult
   {
      final int[] indexBuffer;
      final Vector3f[] normalBuffer;

      private FaceCheckResult(int[] indexBuffer, Vector3f[] normalBuffer)
      {
         this.indexBuffer = indexBuffer;
         this.normalBuffer = normalBuffer;
      }
   }

   private interface ValidPredicate<T> {
      boolean isValid(T vertex);
   }
}
