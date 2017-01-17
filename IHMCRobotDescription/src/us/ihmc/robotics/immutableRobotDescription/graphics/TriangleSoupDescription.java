package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;
import org.jetbrains.annotations.Nullable;
import us.ihmc.graphicsDescription.MeshDataHolder;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

/**
 * Represents an unverified triangle soup that can be used to build a mesh.
 */
@Immutable public abstract class TriangleSoupDescription
{
   public abstract int[] getIndexBuffer();

   public abstract Vector3f[] getVertexBuffer();

   public abstract @Nullable Vector3f[] getNormalBuffer();

   public abstract @Nullable Vector2f[] getTextureCoordinatesBuffer();

   public static TriangleSoupDescription fromMeshDataHolder(MeshDataHolder meshData) {
      Vector3f[] vertices = new Vector3f[meshData.getVertices().length];
      for (int i = 0; i < vertices.length; i++)
      {
         vertices[i] = new Vector3f(meshData.getVertices()[i]);
      }
      Vector2f[] textureCoords = new Vector2f[meshData.getTexturePoints().length];
      for (int i = 0; i < textureCoords.length; i++)
      {
         textureCoords[i] = new Vector2f(meshData.getTexturePoints()[i]);
      }

      return builder()
            .vertexBuffer(vertices)
            .indexBuffer(meshData.getTriangleIndices())
            .normalBuffer(meshData.getVertexNormals())
            .textureCoordinatesBuffer(textureCoords)
            .build();
   }

   public static ImmutableTriangleSoupDescription.Builder builder()
   {
      return ImmutableTriangleSoupDescription.builder();
   }
}
