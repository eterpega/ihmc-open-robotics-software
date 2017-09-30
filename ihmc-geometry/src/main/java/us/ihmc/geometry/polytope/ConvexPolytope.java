package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ConvexPolytope implements GeometryObject<ConvexPolytope>, SupportingVertexHolder
{
   private final ArrayList<PolytopeVertex> vertices = new ArrayList<>();
   private final ArrayList<PolytopeHalfEdge> edges = new ArrayList<>();
   private final ArrayList<PolytopeFace> faces = new ArrayList<>();
   
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   public ConvexPolytope()
   {
      
   }

   public ConvexPolytope(ConvexPolytope polytope)
   {
      set(polytope);
      boundingBoxNeedsUpdating = true;
   }

   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }

      boundingBoxToPack.set(boundingBox);
   }

   private void updateBoundingBox()
   {
      double xMin = Double.POSITIVE_INFINITY;
      double yMin = Double.POSITIVE_INFINITY;
      double zMin = Double.POSITIVE_INFINITY;

      double xMax = Double.NEGATIVE_INFINITY;
      double yMax = Double.NEGATIVE_INFINITY;
      double zMax = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < vertices.size(); i++)
      {
         PolytopeVertex polytopeVertex = vertices.get(i);
         double x = polytopeVertex.getX();
         double y = polytopeVertex.getY();
         double z = polytopeVertex.getZ();

         if (x < xMin)
            xMin = x;
         if (y < yMin)
            yMin = y;
         if (z < zMin)
            zMin = z;

         if (x > xMax)
            xMax = x;
         if (y > yMax)
            yMax = y;
         if (z > zMax)
            zMax = z;
      }

      boundingBox.set(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   public int getNumberOfVertices()
   {
      return vertices.size();
   }
   
   public List<PolytopeVertex> getVertices()
   {
      return vertices;
   }

   public PolytopeVertex getVertex(int index)
   {
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      return edges.size();
   }

   public List<PolytopeHalfEdge> getEdges()
   {
      return edges;
   }
   
   public PolytopeHalfEdge getEdge(int index)
   {
      return edges.get(index);
   }
  
   public int getNumberOfFaces()
   {
      return faces.size();
   }

   public List<PolytopeFace> getFaces()
   {
      return faces;
   }
   
   public PolytopeFace getFace(int index)
   {
      return faces.get(index);
   }
   
   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyTransform(transform);
      }
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).applyInverseTransform(transform);
      }
      boundingBoxNeedsUpdating = true;
   }

   public PolytopeVertex addVertex(double x, double y, double z)
   {
      return addVertex(new PolytopeVertex(x, y, z));
   }
   
   public PolytopeVertex addVertex(Point3D vertexToAdd)
   {
      return addVertex(new PolytopeVertex(vertexToAdd));
   }
   
   public PolytopeVertex addVertex(PolytopeVertex vertexToAdd)
   {
      return null;
   }
   
   @Override
   public Point3D getSupportingVertex(Vector3D supportDirection)
   {
      // Naive implementation. Just search through all of them.
      // TODO: Smart downhill march along edges. But will require always having the edges...

      double maxDotSquared = Double.NEGATIVE_INFINITY;
      PolytopeVertex bestVertex = null;

      int numberOfVertices = vertices.size();
      for (int i = 0; i < numberOfVertices; i++)
      {
         PolytopeVertex vertex = vertices.get(i);
         double dotProduct = vertex.dot(supportDirection);
         if (dotProduct > maxDotSquared)
         {
            maxDotSquared = dotProduct;
            bestVertex = vertex;
         }
      }
      return bestVertex.getPosition();
   }

   public String toString()
   {
      String string = "";

      for (PolytopeVertex vertex : vertices)
      {
         string = string + "\n" + vertex;
      }

      return string;
   }

   @Override
   public boolean epsilonEquals(ConvexPolytope other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean containsNaN()
   {
      boolean result = false;
      for(int i = 0; i < vertices.size(); i++)
      {
         result |= vertices.get(i).containsNaN();
      }
      return result;
   }

   @Override
   public void setToNaN()
   {
      // This should also set all the edges and faces to NaN assuming all relationships are intact
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).setToNaN();;
      }
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and faces to zero assuming all relationships are intact
      for (int i = 0; i < vertices.size(); i++)
      {
         vertices.get(i).setToZero();;
      }
   }

   @Override
   public void set(ConvexPolytope other)
   {
      setVertices(other.getVertices());
      setEdges(other.getEdges());
      setFaces(other.getFaces());
   }
   
   public void setVertices(List<PolytopeVertex> vertices)
   {
      this.vertices.clear();
      this.vertices.addAll(vertices);
   }
   
   public void setEdges(List<PolytopeHalfEdge> edges)
   {
      this.edges.clear();
      this.edges.addAll(edges);
   }
   
   public void setFaces(List<PolytopeFace> faces)
   {
      this.faces.clear();
      this.faces.addAll(faces);
   }
   
   public void clear()
   {
      vertices.clear();
      edges.clear();
      faces.clear();
   }
}
