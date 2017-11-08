package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;

public interface ConvexPolytopeReadOnly extends EpsilonComparable<ConvexPolytopeReadOnly>, SupportingVertexHolder
{
   /**
    * Get a list of faces that consitute the polytope
    * @return a list of read only references to the faces of the polytope
    */
   List<? extends ConvexPolytopeFaceReadOnly> getFaces();

   /**
    * Get a list of half edges that are part of this polytope. List will contain the half edge and its twin
    * Size of the list will be twice the number of edges returned by {@code getNumberOfEdges()}
    * @return a list of read only references to the half edges that make up the faces of this polytope
    */
   List<? extends PolytopeHalfEdgeReadOnly> getEdges();

   /**
    * Get a list of vertices that are part of this polytope. List does not contain any repititions
    * @return a list of read only reference to the vertices of the polytope
    */
   List<? extends PolytopeVertexReadOnly> getVertices();

   /**
    * Returns a reference to the polytope vertex that is further along the direction indicated by the specified vector
    * @param supportingVertexDirection the direction in which the search for said vertex is to be performed
    * @return a read only reference to the required vertex
    */
   PolytopeVertexReadOnly getSupportingPolytopeVertex(Vector3DReadOnly supportingVertexDirection);

   /**
    * Check is the polytope is empty (contains no vertices / edges)
    * @return {@code true} if the polytope has faces that contain edges, otherwise {@code false}
    */
   default boolean isEmpty()
   {
      List<? extends ConvexPolytopeFaceReadOnly> faces = getFaces();
      for (int i = 0; i < faces.size(); i++)
         if (faces.get(i).getNumberOfEdges() != 0)
            return false;
      return true;
   }

   /**
    * Returns the number of vertices that are in the polytope
    * @return integer number of vertices in the polytope
    */
   int getNumberOfVertices();
   
   /**
    * Returns the number of edge that are part of the polytope
    * Note: number of edges is half the number of half edges
    * @return integer number of edges in the face
    */
   int getNumberOfEdges();

   /**
    * Returns the number of face that are constitute the polytope
    * @return integer number of faces 
    */
   int getNumberOfFaces();

   /**
    * Gets a specified face of polytope
    * @param index the index of the polytope. Should be smaller than {@code getNumberOfFaces()}
    * @return
    */
   ConvexPolytopeFaceReadOnly getFace(int index);
}
