package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Template data structure that defines a Doubly-connected edge list (DCEL) polytope vertex
 * 
 * A DCEL vertex is composed of 
 * <li> 3D point: A data structure that stores the spatial location of the vertex in 3D space. May / may not have a notion of a {@code ReferenceFrame}
 * <li> Associated edge list: A list of DCEL edges {@code E} that have their origins at this vertex
 * 
 * @author Apoorv S
 * 
 * @param <V> The class that extends this template. Should represent a point in some 3D space
 * @param <E> Data structure representing edges formed by joining two vertices
 * @param <F> A collection of edges that constitute a face of the polytope
 */
public abstract class PolytopeVertexBasics<V extends PolytopeVertexBasics<V, E, F>, E extends PolytopeHalfEdgeBasics<V, E, F>, F extends ConvexPolytopeFaceBasics<V, E, F>>
      implements SimplexBasics, PolytopeVertexReadOnly, Clearable, Settable<V>, Transformable, Point3DBasics
{
   /**
    * List of edges that start at this vertex. May be part of different faces
    */
   private final ArrayList<E> associatedEdges = new ArrayList<>();

   /**
    * Method to get a reference to the object storing the spatial coordinates for the vertex
    * @return Typically a FramePoint3D or Point3D depending on the use
    */
   protected abstract Point3DBasics getPointObjectReference();

   /**
    * Default constructor
    */
   public PolytopeVertexBasics()
   {
   }

   /**
    * Set the spatial coordinates from another vertex and copy all the edge associations 
    */
   public void set(V other)
   {
      getPointObjectReference().set(other.getPosition());
      clearAssociatedEdgeList();
      addAssociatedEdges(other.getAssociatedEdges());
   }

   /**
    * {@inheritDoc}
    */
   public List<E> getAssociatedEdges()
   {
      return associatedEdges;
   }

   /**
    * {@inheritDoc}
    */
   public E getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   /**
    * Method to remove a particular edge from the associated edge list
    * @param edgeToAdd the associated edge that is to be removed. In case the edge specified is not on the list, no errors are thrown
    * 
    */
   public void removeAssociatedEdge(E edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   /**
    * Remove all edges in the associated edge list
    */
   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   /**
    * Add a {@code List<E>} of DCEL edges to the associated edge list. This invokes the {@code addAssociatedEdge()} method 
    * and addition to the list follows the same set of rules
    * @param edgeList a list of DCEL edges that must be added 
    */
   public void addAssociatedEdges(List<E> edgeList)
   {
      for (int i = 0; i < edgeList.size(); i++)
      {
         addAssociatedEdge(edgeList.get(i));
      }
   }

   /**
    * Add a DCEL edge to the associated edge list. In case the edge is already on the associated edge list
    * no action is carried out. The check for whether an edge is already on the list is done by comparing objects.
    * Hence is possible to add a edge that geometrically equals an already existent edge
    * @param edge the DCEL edge to add to the associated edge list
    */
   public void addAssociatedEdge(E edge)
   {
      if (!isAssociatedWithEdge(edge))
         associatedEdges.add(edge);
   }

   /**
    * {@inheritDoc}
    */
   public boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck)
   {
      return associatedEdges.contains(edgeToCheck);
   }

   /**
    * {@inheritDoc}
    */
   public boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck, double epsilon)
   {
      for (int i = 0; i < associatedEdges.size(); i++)
      {
         if(associatedEdges.get(i).epsilonEquals(edgeToCheck, epsilon))
            return true;
      }
      return false;
   }

   /**
    * {@inheritDoc}
    */
   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   /**
    * {@inheritDoc}
    */
   public double dot(Vector3DReadOnly vector)
   {
      return getX() * vector.getX() + getY() * vector.getY() + getZ() * vector.getZ();
   }

   /**
    * {@inheritDoc}
    */
   public String toString()
   {
      return "( " + getX() + ", " + getY() + ", " + getZ() + ")";
   }

   @Override
   public double getX()
   {
      return getPointObjectReference().getX();
   }

   @Override
   public double getY()
   {
      return getPointObjectReference().getY();
   }

   @Override
   public double getZ()
   {
      return getPointObjectReference().getZ();
   }

   @Override
   public double getElement(int index)
   {
      return getPointObjectReference().getElement(index);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      getPointObjectReference().applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      getPointObjectReference().applyInverseTransform(transform);
   }

   /**
    * {@inheritDoc}
    */
   public boolean isAnyFaceMarked()
   {
      boolean isMarked = false;
      for (int i = 0; !isMarked && i < associatedEdges.size(); i++)
      {
         isMarked |= associatedEdges.get(i).getFace().isMarked();
      }
      return isMarked;
   }

   @Override
   public boolean containsNaN()
   {
      return getPosition().containsNaN();
   }

   @Override
   public void setToNaN()
   {
      getPointObjectReference().setToNaN();
   }

   @Override
   public void setToZero()
   {
      getPointObjectReference().setToZero();
   }

   public void setX(double x)
   {
      getPointObjectReference().setX(x);
   }

   public void setY(double y)
   {
      getPointObjectReference().setY(y);
   }

   public void setZ(double z)
   {
      getPointObjectReference().setZ(z);
   }

   @Override
   public boolean epsilonEquals(PolytopeVertexReadOnly other, double epsilon)
   {
      return getPointObjectReference().epsilonEquals(other, epsilon);
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return getPointObjectReference().distance(point);
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      supportVectorToPack.sub(point, this);
   }

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return this;
   }
   
   public void round(double epsilon)
   {
      setX(MathTools.roundToPrecision(getX(), epsilon * 10));
      setY(MathTools.roundToPrecision(getY(), epsilon * 10));
      setZ(MathTools.roundToPrecision(getZ(), epsilon * 10));
   }
}
