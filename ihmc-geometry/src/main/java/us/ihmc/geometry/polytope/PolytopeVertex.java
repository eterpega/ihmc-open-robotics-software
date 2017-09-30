package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class stores the location of a point which is the vertex of a polytope
 * A list of polytope edges originating from this vertex is also stored for ease of algorithm design 
 * Faces to which this vertex belongs can be accessed by iterating through the list of edges
 * 
 * Based on original class by Jerry Pratt
 * @author Apoorv S
 *
 */
public class PolytopeVertex implements GeometryObject<PolytopeVertex>
{
   private final Point3D position = new Point3D();
   private final ArrayList<PolytopeHalfEdge> associatedEdges = new ArrayList<>();
   
   public PolytopeVertex(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   public PolytopeVertex(Point3D position)
   {
      this.position.set(position);
   }

   public PolytopeVertex(PolytopeVertex vertex)
   {
      this.position.set(vertex.position);
      copyEdges(vertex.getAssociatedEdges());
   }

   @Override
   public void set(PolytopeVertex vertex)
   {
      this.position.set(vertex.position);
      clearEdgeList();
      copyEdges(vertex.getAssociatedEdges());
   }

   public List<PolytopeHalfEdge> getAssociatedEdges()
   {
      return associatedEdges;
   }
   
   public void removeAssociatedEdge(PolytopeHalfEdge edgeToRemove)
   {
      associatedEdges.remove(edgeToRemove);
   }
   
   public void addAssociatedEdge(PolytopeHalfEdge edgeToAdd)
   {
      associatedEdges.add(edgeToAdd);
   }
   
   public void clearEdgeList()
   {
      associatedEdges.clear();
   }

   public void copyEdges(List<PolytopeHalfEdge> edgeList)
   {
      for(int i = 0; i < edgeList.size(); i++)
      {
         addEdge(edgeList.get(i));
      }
   }

   public void addEdge(PolytopeHalfEdge edge)
   {
      if (!containsEdge(edge))
         associatedEdges.add(edge);
   }
   
   public boolean containsEdge(PolytopeHalfEdge edgeToCheck)
   {
      return associatedEdges.contains(edgeToCheck);
   }

   public boolean containsEdge(PolytopeHalfEdge edgeToCheck, double epsilon)
   {
      boolean result = associatedEdges.size() > 0;
      for(int i = 0; result && i < associatedEdges.size(); i++)
      {
         result &= associatedEdges.get(i).epsilonEquals(edgeToCheck, epsilon);
      }
      return result;
   }

   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   public PolytopeHalfEdge getEdge(int index)
   {
      return associatedEdges.get(index);
   }

   public Point3D getPosition()
   {
      return position;
   }

   public double dot(Vector3D vector)
   {
      return position.getX() * vector.getX() + position.getY() * vector.getY() + position.getZ() * vector.getZ();
   }

   public String toString()
   {
      return "{" + position.getX() + ", " + position.getY() + ", " + position.getZ() + "}";
   }

   public double getX()
   {
      return position.getX();
   }

   public double getY()
   {
      return position.getY();
   }

   public double getZ()
   {
      return position.getZ();
   }
   
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(position);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(position);
   }

   @Override
   public boolean epsilonEquals(PolytopeVertex other, double epsilon)
   {
      return position.epsilonEquals(other.position, epsilon);
   }

   @Override
   public boolean containsNaN()
   {
      return position.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      position.setToNaN();
   }

   @Override
   public void setToZero()
   {
      position.setToZero();
   }

}
