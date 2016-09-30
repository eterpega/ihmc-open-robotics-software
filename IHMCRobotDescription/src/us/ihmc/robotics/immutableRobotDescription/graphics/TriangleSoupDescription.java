package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Immutable;
import org.jetbrains.annotations.Nullable;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;

import javax.vecmath.*;
import java.util.*;

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
      int[] polygonIndices = meshData.getPolygonIndices();
      int[] pointsPerPolygonCount = meshData.getPolygonStripCounts();

      ArrayList<Integer> triangleIndices = new ArrayList<>();

      int polygonIndicesStart = 0;
      for (int pointsForThisPolygon : pointsPerPolygonCount)
      {
         int[] polygon = new int[pointsForThisPolygon];

         System.arraycopy(polygonIndices, polygonIndicesStart, polygon, 0, pointsForThisPolygon);

         int[] splitIntoTriangles = splitPolygonIntoTriangles(polygon);

         for (int i : splitIntoTriangles)
         {
            triangleIndices.add(i);
         }

         polygonIndicesStart += pointsForThisPolygon;
      }

      int[] indices = new int[triangleIndices.size()];
      for(int i = 0; i < indices.length; i++)
      {
         indices[i] = triangleIndices.get(i);
      }

      Vector3f[] normals = findNormalsPerVertex(indices, vertices);

      return builder()
            .vertexBuffer(vertices)
            .indexBuffer(indices)
            .normalBuffer(normals)
            .textureCoordinatesBuffer(textureCoords)
            .build();
   }

   private static Vector3f[] findNormalsPerVertex(int[] indices, Tuple3f[] vertices)
   {
      Map<Integer,Set<Integer>> participatingFacesPerVertex = new LinkedHashMap<>();

      Set<Integer> vertexFacesSet;
      for (int i = 0; i < indices.length; i++)
      {
         if(participatingFacesPerVertex.get(indices[i]) == null)
         {
            vertexFacesSet = new LinkedHashSet<>();
            participatingFacesPerVertex.put(indices[i], vertexFacesSet);
         }
         else
         {
            vertexFacesSet = participatingFacesPerVertex.get(indices[i]);
         }

         vertexFacesSet.add(i / 3); // Abuse integer division.
      }

      Vector3f[] normalsPerFace = findNormalsPerFace(indices, vertices);

      int pos = 0;
      Vector3f[] normals = new Vector3f[vertices.length];
      Vector3f vertexNormal, faceNormal;
      for (int vertexIndex = 0; vertexIndex < vertices.length; vertexIndex++)
      {
         Set<Integer> participatingFaceIndices = participatingFacesPerVertex.get(vertexIndex);
         vertexNormal = new Vector3f();
         for (Integer face : participatingFaceIndices)
         {
            faceNormal = normalsPerFace[face];
            vertexNormal.add(faceNormal);
         }

         float faces = (float) participatingFaceIndices.size();
         vertexNormal.scale(1f / faces);
         normals[pos++] = vertexNormal;
      }

      return normals;
   }

   private static Vector3f[] findNormalsPerFace(int[] indices, Tuple3f[] vertices)
   {
      Vector3f[] normalsPerFace = new Vector3f[indices.length / 3]; // Abuse integer division.

      Vector3f firstVector = new Vector3f();
      Vector3f secondVector = new Vector3f();
      Tuple3f[] faceVertices = new Vector3f[3];
      for(int face = 0; face < normalsPerFace.length; face++)
      {
         normalsPerFace[face] = new Vector3f();

         for(int i = 0; i < faceVertices.length; i++)
         {
            faceVertices[i] = vertices[indices[face * 3 + i]];
         }

         firstVector.set(faceVertices[2]);
         firstVector.sub(faceVertices[1]);

         secondVector.set(faceVertices[2]);
         secondVector.sub(faceVertices[0]);

         normalsPerFace[face] = new Vector3f();
         normalsPerFace[face].cross(firstVector, normalsPerFace[face]);

         normalsPerFace[face].normalize();
      }

      return normalsPerFace;
   }

   private static int[] splitPolygonIntoTriangles(int[] polygonIndices)
   {
      if(polygonIndices.length <= 3)
         return polygonIndices;

      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] ret = new int[3 * (polygonIndices.length - 2)];
      int i = 0;
      for(int j = 2; j < polygonIndices.length; j++)
      {
         ret[i++] = polygonIndices[0];
         ret[i++] = polygonIndices[j-1];
         ret[i++] = polygonIndices[j];
      }

      return ret;
   }

   public static ImmutableTriangleSoupDescription.Builder builder()
   {
      return ImmutableTriangleSoupDescription.builder();
   }
}
