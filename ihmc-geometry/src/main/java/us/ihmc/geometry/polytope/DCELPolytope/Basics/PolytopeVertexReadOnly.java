package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface PolytopeVertexReadOnly extends Point3DReadOnly, EpsilonComparable<PolytopeVertexReadOnly>
{
   /**
    * Gets a the spatial location of vertex
    * @return a read only reference to the object that stores the location
    */
   Point3DReadOnly getPosition();

   /**
    * Get list of edges that originate at this vertex
    * @return a list of read only references to the edges that originate at this edge
    */
   List<? extends PolytopeHalfEdgeReadOnly> getAssociatedEdges();

   /**
    * Get a particular associated edge based on its index in the associated edge list held 
    * by the vertex. 
    * @param index must be less than value returned by {@code getNumberOfAssociatedEdges()}
    * @return a read only reference to the edge 
    */
   PolytopeHalfEdgeReadOnly getAssociatedEdge(int index);

   /**
    * Checks if the edge specified originates at the current vertex. Check is performed by comparing objects and not geometrical closeness
    * @param the half edge that is to be checked for association
    * @return {@code true} if the specified edge is on the associated edge list, {@code false} otherwise
    */
   boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck);

   /**
    * An associated edge is sufficiently close if its origin and destination are within a {@code epsilon} distance of the corresponding points for the specified edge
    * @param edgeToCheck
    * @param epsilon
    * @return {@code true} if the specified edge is geometrically close to any of the associated edges. Otherwise {@code false}
    */
   boolean isAssociatedWithEdge(PolytopeHalfEdgeReadOnly edgeToCheck, double epsilon);

   /**
    * Returns the number of references held in the associated edge list 
    * @return integer number of edges that originate at this vertex
    */
   int getNumberOfAssociatedEdges();

   /**
    * Calculates the dot product of the specified vector and the vector from the origin to this vertex
    * @return the resultant dot product
    */
   double dot(Vector3DReadOnly vector);

   /**
    * Get a printable version of the vertex data
    * @return string indicating the spatial coordinates for this vertex
    */
   String toString();

   /**
    * Checks if any of the associated faces for this vertex have been marked
    * Marking is simply a boolean value being set to {@code true} or {@code false} 
    * Useful for some operations that need a temporary list to be stored but you're lazy and don't want 
    * to make that list and look it up all the time 
    * @return {@code true} if the face is not null and marked. Otherwise {@code false}
    */
   boolean isAnyFaceMarked();
}