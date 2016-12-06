package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;
import us.ihmc.graphics3DDescription.HeightMap;

import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;
import java.util.Arrays;

/**
 * Represents a planar grid with an optional height map.
 */
@Immutable
public abstract class HeightMapDescription extends GeometryDescription
{
   public abstract HeightMap getHeightMap();

   /**
    * Number of rows in the triangulated height map.
    * @return row count
    */
   @Default public int getRows() {
      return 32;
   }

   /**
    * Number of columns in the triangulated height map.
    * @return column count
    */
   @Default public int getColumns() {
      return 32;
   }

   private static TriangleMeshDescription createGridGeometry(final int rows, final int cols, HeightMap heightMap)
   {
      int numVertices = rows * cols;
      int numCellRows = rows - 1;
      int numCellCols = cols - 1;

      int numTris = numCellRows * numCellCols * 2;

      Vector3f[] vertices = new Vector3f[numVertices];
      Vector3f[] normals = createNormals(rows, cols, heightMap);
      Vector2f[] textureCoords = new Vector2f[numVertices];

      int k = 0;
      for (float i = 0; i < rows; ++i)
      {

         for (float j = 0; j < cols; ++j)
         {
            vertices[k] = new Vector3f(j / numCellCols, i / numCellRows, 0);
            textureCoords[k] = new Vector2f(vertices[k].x, vertices[k].y);
            ++k;
         }
      }

      int[] indices = new int[numTris * 3];

      // Generate indices for each quad.
      k = 0;
      for (int i = 0; i < numCellRows; ++i)
      {
         for (int j = 0; j < numCellCols; ++j)
         {
            indices[k] = i * cols + j;
            indices[k + 1] = i * cols+ j + 1;
            indices[k + 2] = (i + 1) * cols + j;

            indices[k + 3] = (i + 1) * cols + j;
            indices[k + 4] = i * cols + j + 1;
            indices[k + 5] = (i + 1) * cols + j + 1;

            k += 6;
         }
      }

      return TriangleMeshDescription.fromTriangleSoupSanitized(
            TriangleSoupDescription.builder()
                  .indexBuffer(indices)
                  .vertexBuffer(vertices)
                  .normalBuffer(normals)
                  .textureCoordinatesBuffer(textureCoords)
                  .build());
   }

   private static Vector3f[] createNormals(int rows, int cols, HeightMap heightMap) {
      Vector3f[] normalsPerSquare = new Vector3f[(rows + 1) * (cols + 1)]; // fill with default normal around to make bounds check easier
      float xExtent = (float)(heightMap.getBoundingBox().getZMax() - heightMap.getBoundingBox().getZMin());
      float yExtent = (float)(heightMap.getBoundingBox().getYMax() - heightMap.getBoundingBox().getYMin());
      float columnWidth = 1f / (cols - 1);
      float rowHeight = 1f / (rows - 1);
      float mapOffsetX = xExtent / (cols - 1);
      float mapOffsetY = yExtent / (rows - 1);
      Arrays.fill(normalsPerSquare, 0, normalsPerSquare.length - 1, new Vector3f(0, 0, 1));
      for (int row = 1; row < rows - 1; row++)
      {
         for (int col = 1; col < cols - 1; col++)
         {
            float xCenter = (float)col / (cols - 1) + columnWidth / 2;
            float yCenter = (float)row / (rows - 1) + rowHeight / 2;
            float heightCenter = sampleHeightMap(xCenter, yCenter, heightMap);
            float heightLeft = sampleHeightMap(xCenter - columnWidth, yCenter, heightMap);
            float heightRight = sampleHeightMap(xCenter + columnWidth, yCenter, heightMap);
            float heightTop = sampleHeightMap(xCenter, yCenter - rowHeight, heightMap);
            float heightBottom = sampleHeightMap(xCenter, yCenter + rowHeight, heightMap);

            Vector3f tangentLeft = new Vector3f(-mapOffsetX, 0, heightCenter - heightLeft);
            Vector3f tangentRight = new Vector3f(mapOffsetX, 0, heightCenter - heightRight);
            Vector3f tangentTop = new Vector3f(0, -mapOffsetY, heightCenter - heightTop);
            Vector3f tangentBottom = new Vector3f(0, mapOffsetY, heightCenter - heightBottom);

            Vector3f normal1 = new Vector3f(); normal1.cross(tangentTop, tangentLeft);
            Vector3f normal2 = new Vector3f(); normal2.cross(tangentRight, tangentBottom);
            if (normal1.lengthSquared() < 1e-8)
               normal1 = new Vector3f(0, 0, 1);
            if (normal2.lengthSquared() < 1e-8)
               normal2 = new Vector3f(0, 0, 1);
            normal1.normalize();
            normal2.normalize();
            Vector3f finalNormal = new Vector3f();
            finalNormal.add(normal1);
            finalNormal.add(normal2);
            finalNormal.normalize();

            normalsPerSquare[col + row * (cols + 1)] = finalNormal;
         }
      }

      Vector3f[] normalsPerPoint = new Vector3f[rows * cols];
      for (int row = 0; row < rows; row++)
      {
         for (int col = 0; col < cols; col++)
         {
            Vector3f topLeft = normalsPerSquare[col + (row * (cols + 1))];
            Vector3f topRight = normalsPerSquare[col + (row * (cols + 1))];
            Vector3f bottomLeft = normalsPerSquare[col + (row * (cols + 1))];
            Vector3f bottomRight = normalsPerSquare[col + (row * (cols + 1))];
            Vector3f normal = new Vector3f();
            normal.add(topLeft);
            normal.add(topRight);
            normal.add(bottomLeft);
            normal.add(bottomRight);
            normal.scale(1f / 4);
            normal.normalize();
            normalsPerPoint[col + row * cols] = normal;
         }
      }
      return normalsPerPoint;
   }

   private static float sampleHeightMap(float x, float y, HeightMap heightMap) {
      float xExtent = (float)(heightMap.getBoundingBox().getZMax() - heightMap.getBoundingBox().getZMin());
      float xMin = (float)heightMap.getBoundingBox().getXMin();
      float yExtent = (float)(heightMap.getBoundingBox().getYMax() - heightMap.getBoundingBox().getYMin());
      float yMin = (float)heightMap.getBoundingBox().getYMin();
      return (float)heightMap.heightAt(x * xExtent - xMin, y * yExtent - yMin, 0);
   }

   @Override public TriangleGeometryDescription toTriangleGeometry()
   {
      return TriangleGeometryDescription.builder()
            .triangleMesh(createGridGeometry(getRows(), getColumns(), getHeightMap()))
            .transform(getTransform())
            .build();
   }

   public static ImmutableHeightMapDescription.Builder builder()
   {
      return ImmutableHeightMapDescription.builder();
   }
}
