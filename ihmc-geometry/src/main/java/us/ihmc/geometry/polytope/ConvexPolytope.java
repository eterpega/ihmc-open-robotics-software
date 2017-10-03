package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ConvexPolytope implements GeometryObject<ConvexPolytope>, SupportingVertexHolder
{
   /*
    *  Convex polytope holds vertices and edges for faster computation.
    *  Unless there is reason to believe operations on vertices / edges will be faster, the convex polytope math will use faces since they provide more information (connectivity etc.)
    */
   
   /**
    * Vertices of the polytope
    */
   private final RecyclingArrayList<PolytopeVertex> vertices = new RecyclingArrayList<>(PolytopeVertex.class);
   /**
    * Edges of the polytope
    */
   private final RecyclingArrayList<PolytopeHalfEdge> edges = new RecyclingArrayList<>(PolytopeHalfEdge.class);
   /**
    * Faces of the polytope
    */
   private final RecyclingArrayList<PolytopeFace> faces = new RecyclingArrayList<>(PolytopeFace.class);
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

   // Temporary variables for intermediate results
   private Vector3D tempVector = new Vector3D();
   
   public ConvexPolytope()
   {
      // Default constructor 
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
      // Edges are twinned
      return (edges.size() / 2);
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
      return addVertex(vertexToAdd, Epsilons.ONE_TEN_THOUSANDTH);
   }
   
   // This is a garbage creating implementation. Fix if possible
   public PolytopeVertex addVertex(PolytopeVertex vertexToAdd, double epsilon)
   {
      switch (vertices.size())
      {
      // Create a new face since the initial polytope has zero faces and add the vertex
      case 0: faces.add(new PolytopeFace());
      case 1:
      case 2:
         // Let the face handle all cases that define the first plane of the polytope
         vertices.add(vertexToAdd);
         edges.clear();
         faces.get(0).addVertex(vertexToAdd);
         edges.addAll(faces.get(0).getEdgeList());
         break;
      default:
         if(faces.size() == 1)
         {
            //Handle the degenerate case where we the convex hull is still a plane
            PolytopeFace faceUnderConsideration = faces.get(0);
            if(faceUnderConsideration.isInteriorPoint(vertexToAdd))
            {
               faceUnderConsideration.addVertex(vertexToAdd);
               vertices.add(vertexToAdd);
            }
            else
            {
               visibleEdgeList.clear();
               visibleEdgeList.addAll(faceUnderConsideration.getEdgeList());
               createFacesFromVisibleSilhouette(vertexToAdd, visibleEdgeList);
            }
         }
         else
         {
            //TODO 
         }
         // Check if the point is an interior point. Do nothing in case it is
         
         break;
      }
      return vertexToAdd;
   }
   
   private void createFacesFromVisibleSilhouette(PolytopeVertex vertex, List<PolytopeHalfEdge> visibleEdgeList)
   {
      
   }
   
   private List<PolytopeHalfEdge> visibleEdgeList = new ArrayList<>(1000);
   
   public void getVisibleSilhouette(Point3D pointUnderConsideration, List<PolytopeHalfEdge> visibleEdgeListToPack)
   {
      double epsilon = Epsilons.ONE_TEN_THOUSANDTH;
      visibleEdgeListToPack.clear();
      PolytopeFace faceUnderConsideration = isInteriorPointInternal(pointUnderConsideration, epsilon);
      PolytopeHalfEdge edgeUnderConsideration = faceUnderConsideration.getEdge(0);
      if(faceUnderConsideration != null)
      {
         while(true)
         {
            for(int i = 0; i < faceUnderConsideration.getNumberOfEdges(); i++)
            {
               if(edgeUnderConsideration.getTwinHalfEdge() == null)
               {
                  //TODO 
               }
               else
               {
                  tempVector.sub(pointUnderConsideration, edgeUnderConsideration.getDestinationVertex().getPosition());
                  if(tempVector.dot(edgeUnderConsideration.getTwinHalfEdge().getFace().getFaceNormal()) <=0)
                     break;
                  else
                  {
                     
                  }
               }
               edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
            }
            // Switch to the next face
            edgeUnderConsideration = edgeUnderConsideration.getTwinHalfEdge();
            faceUnderConsideration = edgeUnderConsideration.getFace();
         }
      }
   }

   private PolytopeFace isInteriorPointInternal(Point3D pointToCheck, double epsilon)
   {
      for(int i = 0; i < faces.size(); i++)
      {
         tempVector.sub(pointToCheck, faces.get(i).getEdge(0).getOriginVertex().getPosition());
         double dotProduct = tempVector.dot(faces.get(i).getFaceNormal());
         if(dotProduct >= -epsilon)
         {
            return faces.get(i);
         }
      }
      return null;
   }
   
   public boolean isInteriorPoint(Point3D pointToCheck, double epsilon)
   {
      return isInteriorPointInternal(pointToCheck, epsilon) == null;
   }
   
   @Override
   public Point3D getSupportingVertex(Vector3D supportDirection)
   {
      // Naive implementation. Just search through all of them.
      // TODO: Smart downhill march along edges. We now have the edges

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
      }e new vertices, edge
      s, and faces, to be added
      to the corresponding list within the DCEL and—symmetricall
      y—the ability to delete an
      existing entity. Then it should be easy to add a new vertex
      v
      to the graph within some
      face
      f
      . As we maintain a connected graph, we better link the new vert
      ex to somewhere,
      say, to an existing vertex
      u
      . For such a connection to be possible, we require that the
      open line segment
      uv
      lies completely in
      f

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
   
   private void setVertices(List<PolytopeVertex> vertices)
   {
      this.vertices.clear();
      this.vertices.addAll(vertices);
   }
   
   private void setEdges(List<PolytopeHalfEdge> edges)
   {
      this.edges.clear();
      this.edges.addAll(edges);
   }
   
   private void setFaces(List<PolytopeFace> faces)
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
