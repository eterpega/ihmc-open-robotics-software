package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * A convex polytope is a collection of faces that describe it 
 * 
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ConvexPolytope implements GeometryObject<ConvexPolytope>, SupportingVertexHolder
{
   private final ArrayList<PolytopeHalfEdge> edges = new ArrayList<>();
   private final ArrayList<PolytopeFace> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<PolytopeFace> markedList = new ArrayList<>();
   private final ArrayList<PolytopeFace> onFaceList = new ArrayList<>();
   private final ArrayList<PolytopeHalfEdge> visibleSilhouetteList = new ArrayList<>();

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

      for (int i = 0; i < faces.size(); i++)
      {
         double x = faces.get(i).getMinX();
         double y = faces.get(i).getMinY();
         double z = faces.get(i).getMinZ();

         if (x < xMin)
            xMin = x;
         if (y < yMin)
            yMin = y;
         if (z < zMin)
            zMin = z;

         x = faces.get(i).getMaxX();
         y = faces.get(i).getMaxY();
         z = faces.get(i).getMaxZ();
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
      // Polyhedron formula
      return getNumberOfEdges() - getNumberOfFaces() + 2;
   }

   public List<PolytopeVertex> getVertices()
   {
      // TODO implement
      throw new RuntimeException("Method not implemented and may never be");
   }

   public PolytopeVertex getVertex(int index)
   {
      // TODO implement 
      throw new RuntimeException("Method not implemented and may never be");
   }

   public int getNumberOfEdges()
   {
      updateEdges();
      return edges.size() / 2;
   }

   public List<PolytopeHalfEdge> getEdges()
   {
      updateEdges();
      return edges;
   }

   private void updateEdges()
   {
      edges.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         List<PolytopeHalfEdge> faceEdgeList = faces.get(i).getEdgeList();
         for (int j = 0; j < faceEdgeList.size(); j++)
         {
            edges.add(faceEdgeList.get(j));
         }
      }
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
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).applyTransform(transform);
      }
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).applyInverseTransform(transform);
      }
      boundingBoxNeedsUpdating = true;
   }

   public void addVertex(double... coordinates)
   {
      addVertex(new PolytopeVertex(coordinates[0], coordinates[1], coordinates[2]));
   }

   public void addVertex(double x, double y, double z)
   {
      addVertex(new PolytopeVertex(x, y, z));
   }

   public void addVertex(Point3D vertexToAdd)
   {
      addVertex(new PolytopeVertex(vertexToAdd));
   }

   /**
    * Adds a polytope vertex to the current polytope. 
    * In case needed faces are removed and recreated. This will result in garbage. Fix if possible
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public void addVertex(PolytopeVertex vertexToAdd)
   {
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         PolytopeFace newFace = new PolytopeFace();
         newFace.addVertex(vertexToAdd);
         faces.add(newFace);
         return;
      }
      else if (faces.size() == 1)
      {
         if (faces.get(0).isPointInFacePlane(vertexToAdd, Epsilons.ONE_MILLIONTH))
         {
            if (faces.get(0).isInteriorPoint(vertexToAdd))
               return;
            else
               faces.get(0).addVertex(vertexToAdd);
         }
         else
         {
            if (faces.get(0).isFaceVisible(vertexToAdd))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            visibleSilhouetteList.addAll(faces.get(0).getEdgeList());
            createFacesFromVisibleSilhouette(vertexToAdd);
         }
         return;
      }

      getVisibleFaces(vertexToAdd, markedList);
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, Epsilons.ONE_BILLIONTH);

      // Delete faces that have all surrounding faces in the marking list. These are not useful now 
      for (int i = 0; i < markedList.size();)
      {
         PolytopeFace candidateFace = markedList.get(i);

         boolean allNeighbouringFacesAreMarkedForDeletion = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesAreMarkedForDeletion &= markedList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesAreMarkedForDeletion)
         {
            removeFace(candidateFace);
            markedList.remove(candidateFace);
         }
         else
            i++;
      }
      // Now marking list contains the faces whose edges will create the visible silhouette. Get that edge list

      // Smartly get one silhouette edge that is after a half edge whose twin we just removed
      PolytopeFace faceUnderConsideration = markedList.get(0);
      PolytopeHalfEdge firstHalfEdgeForSilhouette = faceUnderConsideration.getEdge(0);
      for (int i = 0; i < faceUnderConsideration.getNumberOfEdges(); i++)
      {
         if (firstHalfEdgeForSilhouette.getTwinHalfEdge() == null && firstHalfEdgeForSilhouette.getNextHalfEdge().getTwinHalfEdge() != null)
            break;
         firstHalfEdgeForSilhouette = firstHalfEdgeForSilhouette.getNextHalfEdge();
      }
      firstHalfEdgeForSilhouette = firstHalfEdgeForSilhouette.getNextHalfEdge();
      
      // Follow the trail of silhouette edges and keep deleting the faces that we are done with
      PolytopeHalfEdge halfEdgeUnderConsideration = firstHalfEdgeForSilhouette;
      visibleSilhouetteList.clear();

      while(true)
      {
         visibleSilhouetteList.add(halfEdgeUnderConsideration.getTwinHalfEdge());
         if(halfEdgeUnderConsideration.getNextHalfEdge().getTwinHalfEdge() != null && markedList.contains(halfEdgeUnderConsideration.getNextHalfEdge().getTwinHalfEdge().getFace()) )
            halfEdgeUnderConsideration = halfEdgeUnderConsideration.getNextHalfEdge().getTwinHalfEdge().getNextHalfEdge();
         else
            halfEdgeUnderConsideration = halfEdgeUnderConsideration.getNextHalfEdge();
         if(halfEdgeUnderConsideration == firstHalfEdgeForSilhouette)
            break;
      }
      
//      for (int i = 0; i < markedList.size(); i++)
//      {
//         for(int j = 0; j < markedList.get(i).getNumberOfEdges() && !markedList.contains(halfEdgeUnderConsideration.getTwinHalfEdge().getFace()); j++) 
//         {
//            PrintTools.debug(halfEdgeUnderConsideration.getTwinHalfEdge().toString());
//            visibleSilhouetteList.add(halfEdgeUnderConsideration.getTwinHalfEdge());
//            halfEdgeUnderConsideration = halfEdgeUnderConsideration.getNextHalfEdge();
//         }
//         halfEdgeUnderConsideration = halfEdgeUnderConsideration.getTwinHalfEdge().getNextHalfEdge();
//      }
      removeMarkedFaces();
      createFacesFromVisibleSilhouette(vertexToAdd);
   }

   private void createFacesFromVisibleSilhouette(PolytopeVertex vertexToAdd)
   {
      PolytopeFace firstNewFace = new PolytopeFace();
      firstNewFace.addVertex(visibleSilhouetteList.get(0).getDestinationVertex());
      firstNewFace.addVertex(visibleSilhouetteList.get(0).getOriginVertex());
      firstNewFace.addVertex(vertexToAdd);
      visibleSilhouetteList.get(0).setTwinHalfEdge(firstNewFace.getEdge(0));
      firstNewFace.getEdge(0).setTwinHalfEdge(visibleSilhouetteList.get(0));
      faces.add(firstNewFace);
      for (int i = 1; i < visibleSilhouetteList.size(); i++)
      {
         PolytopeFace newFace = new PolytopeFace();
         faces.add(newFace);
         newFace.addVertex(visibleSilhouetteList.get(i).getDestinationVertex());
         newFace.addVertex(visibleSilhouetteList.get(i).getOriginVertex());
         newFace.addVertex(vertexToAdd);
         
         visibleSilhouetteList.get(i).setTwinHalfEdge(newFace.getEdge(0));
         newFace.getEdge(0).setTwinHalfEdge(visibleSilhouetteList.get(i));

         visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge().setTwinHalfEdge(newFace.getEdge(0).getPreviousHalfEdge());
         newFace.getEdge(0).getPreviousHalfEdge().setTwinHalfEdge(visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge());
      }
      firstNewFace.getEdge(0).getPreviousHalfEdge()
                  .setTwinHalfEdge(visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge());
      visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge()
                           .setTwinHalfEdge(firstNewFace.getEdge(0).getPreviousHalfEdge());
   }

   private void removeMarkedFaces()
   {
      for (int i = 0; i < markedList.size(); i++)
      {
         removeFace(markedList.get(i));
      }
   }

   public void getVisibleFaces(Point3DBasics vertexUnderConsideration, List<PolytopeFace> faceReferencesToPack)
   {
      faceReferencesToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isFaceVisible(vertexUnderConsideration))
         {
            faceReferencesToPack.add(faces.get(i));
         }
      }
   }

   public void getFacesWhichPointIsOn(Point3DBasics vertexUnderConsideration, List<PolytopeFace> faceReferenceToPack, double epsilon)
   {
      faceReferenceToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isPointInFacePlane(vertexUnderConsideration, epsilon))
         {
            faceReferenceToPack.add(faces.get(i));
         }
      }
   }

   public void removeFace(PolytopeFace faceToRemove)
   {
      for (int i = 0; i < faceToRemove.getNumberOfEdges(); i++)
      {
         PolytopeHalfEdge twinHalfEdge = faceToRemove.getEdge(i).getTwinHalfEdge();
         if (twinHalfEdge != null)
            twinHalfEdge.setTwinHalfEdge(null);
         faceToRemove.getEdge(i).clear();
      }
      faceToRemove.clearEdgeList();
      faces.remove(faceToRemove);
   }

   private PolytopeFace isInteriorPointInternal(Point3D pointToCheck, double epsilon)
   {
      for (int i = 0; i < faces.size(); i++)
      {
         tempVector.sub(pointToCheck, faces.get(i).getEdge(0).getOriginVertex().getPosition());
         double dotProduct = tempVector.dot(faces.get(i).getFaceNormal());
         if (dotProduct >= -epsilon)
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
      PolytopeFace bestFace = faces.get(0);
      PolytopeFace bestFaceCandidate = faces.get(0);
      double maxDot = supportDirection.dot(bestFaceCandidate.getFaceNormal());
      while (true)
      {
         for (int i = 0; i < bestFace.getNumberOfEdges(); i++)
         {
            double dotCandidate = supportDirection.dot(bestFace.getNeighbouringFace(i).getFaceNormal());
            if (maxDot < dotCandidate)
            {
               maxDot = dotCandidate;
               bestFaceCandidate = bestFace.getNeighbouringFace(i);
            }
         }
         if (bestFace == bestFaceCandidate)
         {
            return bestFace.getSupportingVertex(supportDirection);
         }
         else
            bestFace = bestFaceCandidate;
      }
   }

   public String toString()
   {
      String string = "Number of faces: " + faces.size();
      for (int i = 0; i < faces.size(); i++)
      {
         string = string + "\n" + faces.get(i).toString();
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
      for (int i = 0; i < faces.size(); i++)
      {
         result |= faces.get(i).containsNaN();
      }
      return result;
   }

   @Override
   public void setToNaN()
   {
      // This should also set all the edges and vertices to NaN assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToNaN();
         ;
      }
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and vertices to zero assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToZero();
         ;
      }
   }

   @Override
   public void set(ConvexPolytope other)
   {
      setFaces(other.getFaces());
   }

   private void setFaces(List<PolytopeFace> faces)
   {
      this.faces.clear();
      this.faces.addAll(faces);
   }

   public void clear()
   {
      edges.clear();
      faces.clear();
   }
}
