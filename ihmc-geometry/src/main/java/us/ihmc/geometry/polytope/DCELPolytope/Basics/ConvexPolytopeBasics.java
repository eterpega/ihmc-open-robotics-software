package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

public abstract class ConvexPolytopeBasics<T extends PolytopeVertexBasics<T, S, U, Q>, S extends PolytopeHalfEdgeBasics<T, S, U, Q>, U extends ConvexPolytopeFaceBasics<T, S, U, Q>, V extends ConvexPolytopeBasics<T, S, U, V, Q>, Q extends SimplexBasics<Q>>
      implements GeometryObject<V>, SimplexBasics<Q>, SupportingVertexHolder
{
   private final ArrayList<T> vertices = new ArrayList<>();
   private final ArrayList<S> edges = new ArrayList<>();
   private final ArrayList<U> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<U> visibleFaces = new ArrayList<>();
   private final ArrayList<U> silhouetteFaces = new ArrayList<>();
   private final ArrayList<U> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<U> onFaceList = new ArrayList<>();
   private final ArrayList<S> visibleSilhouetteList = new ArrayList<>();

   // Temporary variables for intermediate results
   private Vector3D tempVector = new Vector3D();
   private Point3D centroid = new Point3D();

   public ConvexPolytopeBasics()
   {
      
   }
   
   public ConvexPolytopeBasics(V polytope)
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
      // Polyhedron formula for quick calc
      return getNumberOfEdges() - getNumberOfFaces() + 2;
   }

   public List<T> getVertices()
   {
      updateVertices();
      return vertices;
   }

   private void updateVertices()
   {
      unmarkAllFaces();
      vertices.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < faces.get(i).getNumberOfEdges(); j++)
         {
            if (!faces.get(i).getEdge(j).getOriginVertex().isAnyFaceMarked())
            {
               vertices.add(faces.get(i).getEdge(j).getOriginVertex());
            }
         }
         faces.get(i).mark();
      }
   }

   public T getVertex(int index)
   {
      updateVertices();
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      updateEdges();
      return edges.size() / 2;
   }

   public List<S> getEdges()
   {
      updateEdges();
      return edges;
   }

   private void updateEdges()
   {
      edges.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         List<S> faceEdgeList = faces.get(i).getEdgeList();
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

   public List<U> getFaces()
   {
      return faces;
   }

   public U getFace(int index)
   {
      return faces.get(index);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // Applying the transform to the vertices is less expensive computationally but getting the vertices is hard
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         vertices.get(i).applyInverseTransform(transform);
      boundingBoxNeedsUpdating = true;
   }

   private void unmarkAllFaces()
   {
      for (int i = 0; i < faces.size(); i++)
         faces.get(i).unmark();
   }

   public void addVertices(double epsilon, Point3D... vertices)
   {
      for (int i = 0; i < vertices.length; i++)
         addVertex(vertices[i], epsilon);
   }

   public void addVertices(List<T> vertices, double epsilon)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertex(double epsilon, double... coordinates)
   {
      addVertex(getVertexProvider().getVertex(coordinates[0], coordinates[1], coordinates[2]), epsilon);
   }

   public void addVertex(double x, double y, double z, double epsilon)
   {
      addVertex(getVertexProvider().getVertex(x, y, z), epsilon);
   }

   public void addVertex(Point3D vertexToAdd, double epsilon)
   {
      addVertex(getVertexProvider().getVertex(vertexToAdd), epsilon);
   }

   /**
    * Adds a polytope vertex to the current polytope. 
    * In case needed faces are removed and recreated. This will result in garbage. Fix if possible
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public void addVertex(T vertexToAdd, double epsilon)
   {
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         U newFace = getConvexFaceProvider().getFace();
         newFace.addVertex(vertexToAdd);
         faces.add(newFace);
         boundingBoxNeedsUpdating = true;
         return;
      }
      else if (faces.size() == 1)
      {
         if (faces.get(0).isPointInFacePlane(vertexToAdd, Epsilons.ONE_MILLIONTH))
         {
            if (!faces.get(0).isInteriorPoint(vertexToAdd))
               faces.get(0).addVertex(vertexToAdd);
            return;
         }
         else
         {
            if (faces.get(0).isFaceVisible(vertexToAdd, epsilon))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            S halfEdge = faces.get(0).getEdge(0);
            for (int i = 0; i < faces.get(0).getNumberOfEdges(); i++)
            {
               visibleSilhouetteList.add(halfEdge);
               halfEdge = halfEdge.getPreviousHalfEdge();
            }
            onFaceList.clear();
            createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd);
         }
         boundingBoxNeedsUpdating = true;
         return;
      }

      U visibleFaceSeed = getVisibleFaces(visibleFaces, vertexToAdd, epsilon);
      if (visibleFaces.isEmpty())
         return;
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, Epsilons.ONE_BILLIONTH);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      S firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertexToAdd).getTwinHalfEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, visibleFaceSeed);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, visibleFaces);
      removeFaces(nonSilhouetteFaces);
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd);
      boundingBoxNeedsUpdating = true;
   }

   private double getTripleProduct(Vector3DReadOnly v1, Vector3DReadOnly v2, Vector3DReadOnly v3)
   {
      tempVector.cross(v1, v2);
      return tempVector.dot(v3);
   }

   private S getCommonEdge(U face1, U face2)
   {
      S edgeCandidate = null;
      tempVector.cross(face1.getFaceNormal(), face2.getFaceNormal());
      tempVector.normalize();
      for (int i = 0; i < face1.getNumberOfEdges(); i++)
      {
         if (tempVector.dot(face1.getEdge(i).getNormalizedEdgeVector()) >= 1.0)
         {
            edgeCandidate = face1.getEdge(i);
            break;
         }
      }
      return edgeCandidate;
   }

   public void getSilhouetteFaces(List<U> silhouetteFacesToPack, List<U> nonSilhouetteFacesToPack,
                                  List<U> visibleFaceList)
   {
      if (silhouetteFacesToPack != null)
         silhouetteFacesToPack.clear();
      if (nonSilhouetteFacesToPack != null)
         nonSilhouetteFacesToPack.clear();
      for (int i = 0; i < visibleFaceList.size(); i++)
      {
         U candidateFace = visibleFaceList.get(i);

         boolean allNeighbouringFacesVisible = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesVisible &= visibleFaceList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesVisible && nonSilhouetteFacesToPack != null)
            nonSilhouetteFacesToPack.add(candidateFace);
         else if (silhouetteFacesToPack != null)
            silhouetteFacesToPack.add(candidateFace);
      }
   }

   public void getVisibleSilhouette(Point3DReadOnly vertex, List<S> visibleSilhouetteToPack, double epsilon)
   {
      U leastVisibleFace = getVisibleFaces(visibleFaces, vertex, epsilon);
      if (visibleFaces.isEmpty())
         return;
      getFacesWhichPointIsOn(vertex, onFaceList, Epsilons.ONE_BILLIONTH);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      S firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertex).getTwinHalfEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, leastVisibleFace);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, firstHalfEdgeForSilhouette, visibleFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<S> visibleSilhouetteToPack, S seedHalfEdge,
                                             List<U> silhouetteFaceList)
   {
      S halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      int numberOfEdges = getNumberOfEdges();
      int count;
      for (count = 0; count < numberOfEdges; count++)
      {
         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinHalfEdge());
         T destinationVertex = halfEdgeUnderConsideration.getDestinationVertex();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            if (silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getFace())
                  && !silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge().getFace()))
            {
               halfEdgeUnderConsideration = destinationVertex.getAssociatedEdge(i);
               break;
            }
         }
         if (halfEdgeUnderConsideration == seedHalfEdge)
            break;
      }
      if (count == numberOfEdges)
         PrintTools.warn("Could not determine visible silhouette \n" + toString());
   }

   public S getSeedEdgeForSilhouetteCalculation(List<U> visibleFaceList, U leastVisibleFace)
   {
      if (faces.size() == 1)
         return faces.get(0).getEdge(0);
      S seedEdge = null;
      S seedEdgeCandidate = leastVisibleFace.getEdge(0);
      for (int i = 0; seedEdge == null && i < leastVisibleFace.getNumberOfEdges(); i++)
      {
         if (!visibleFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace()))
            seedEdge = seedEdgeCandidate;
         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
      }
      return seedEdge;
   }

   //   public S getSeedEdgeForSilhouetteCalculation(List<U> silhouetteFaceList)
   //   {
   //      U seedFaceCandidate = silhouetteFaceList.get(0);
   //      S seedEdgeCandidate = seedFaceCandidate.getEdge(0);
   //      for (int i = 0; i < seedFaceCandidate.getNumberOfEdges(); i++)
   //      {
   //         if (silhouetteFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace())
   //               && !silhouetteFaceList.contains(seedEdgeCandidate.getNextHalfEdge().getTwinHalfEdge().getFace()))
   //            break;
   //         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      }
   //      seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
   //      return seedEdgeCandidate;
   //   }

   //   private void createFacesFromVisibleSilhouette(PolytopeVertex vertexToAdd)
   //   {
   //      U firstNewFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(0));
   //      twinEdges(visibleSilhouetteList.get(0), firstNewFace.getEdge(0));
   //      for (int i = 1; i < visibleSilhouetteList.size(); i++)
   //      {
   //         U newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(i));
   //         twinEdges(visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge(), newFace.getEdge(0).getPreviousHalfEdge());
   //      }
   //      twinEdges(visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge(), firstNewFace.getEdge(0).getPreviousHalfEdge());
   //   }

   private void createFacesFromVisibleSilhouetteAndOnFaceList(List<S> silhouetteEdges, List<U> onFaceList, T vertexToAdd)
   {
      //for(int i = 0; i < silhouetteEdges.size(); i++)
      //PrintTools.debug("Sil: " + silhouetteEdges.get(i));
      //PrintTools.debug(silhouetteEdges.get(0).getFace().toString());
      S previousLeadingEdge = null, trailingEdge = null;
      if (onFaceList.contains(silhouetteEdges.get(0).getFace()))
      {
         previousLeadingEdge = silhouetteEdges.get(0).getFace().getFirstVisibleEdge(vertexToAdd);
         silhouetteEdges.get(0).getFace().addVertex(vertexToAdd);
         trailingEdge = previousLeadingEdge.getNextHalfEdge();
      }
      else
      {
         U firstFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(0));
         previousLeadingEdge = firstFace.getEdge(0).getNextHalfEdge();
         trailingEdge = firstFace.getEdge(0).getPreviousHalfEdge();
         //PrintTools.debug("This happened");
      }
      //PrintTools.debug("PrevLeadEdge: "  + ((previousLeadingEdge == null )? "null" : previousLeadingEdge.toString()));
      //PrintTools.debug("TrailEdge: "  + ((trailingEdge == null) ? "null" : trailingEdge.toString() ));
      for (int i = 1; i < silhouetteEdges.size(); i++)
      {
         //PrintTools.debug("Previous leading: " + previousLeadingEdge.toString() + " Visible : " + visibleSilhouetteList.get(i).toString());
         if (onFaceList.contains(silhouetteEdges.get(i).getFace()))
         {
            S tempEdge = silhouetteEdges.get(i).getFace().getFirstVisibleEdge(vertexToAdd);
            silhouetteEdges.get(i).getFace().addVertex(vertexToAdd);
            twinEdges(previousLeadingEdge, tempEdge.getNextHalfEdge());
            previousLeadingEdge = tempEdge;
         }
         else
         {
            U newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(i));
            twinEdges(previousLeadingEdge, newFace.getEdge(0).getPreviousHalfEdge());
            previousLeadingEdge = newFace.getEdge(0).getNextHalfEdge();
         }
      }
      twinEdges(previousLeadingEdge, trailingEdge);
   }

   private void twinEdges(S halfEdge1, S halfEdge2)
   {
      halfEdge1.setTwinHalfEdge(halfEdge2);
      halfEdge2.setTwinHalfEdge(halfEdge1);
   }

   private U createFaceFromTwinEdgeAndVertex(T vertex, S twinEdge)
   {
      U newFace = getConvexFaceProvider().getFace();
      faces.add(newFace);
      newFace.addVertex(twinEdge.getDestinationVertex());
      newFace.addVertex(twinEdge.getOriginVertex());
      newFace.addVertex(vertex);
      twinEdges(newFace.getEdge(0), twinEdge);
      return newFace;
   }

   private void removeFaces(List<U> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
   }

   public U getVisibleFaces(List<U> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      U leastVisibleFace = null;
      double minimumVisibilityProduct = Double.POSITIVE_INFINITY;
      faceReferencesToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         double visibilityProduct = faces.get(i).getFaceVisibilityProduct(vertexUnderConsideration);
         if (visibilityProduct > epsilon)
         {
            faceReferencesToPack.add(faces.get(i));
            if (visibilityProduct < minimumVisibilityProduct)
            {
               leastVisibleFace = faces.get(i);
               minimumVisibilityProduct = visibilityProduct;
            }
         }
      }
      return leastVisibleFace;
   }

   public void getFacesWhichPointIsOn(Point3DReadOnly vertexUnderConsideration, List<U> faceReferenceToPack, double epsilon)
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

   public void removeFace(U faceToRemove)
   {
      for (int i = 0; i < faceToRemove.getNumberOfEdges(); i++)
      {
         S twinHalfEdge = faceToRemove.getEdge(i).getTwinHalfEdge();
         if (twinHalfEdge != null)
            twinHalfEdge.setTwinHalfEdge(null);
         faceToRemove.getEdge(i).clear();
      }
      faceToRemove.clearEdgeList();
      faces.remove(faceToRemove);
   }

   private U isInteriorPointInternal(Point3DReadOnly pointToCheck, double epsilon)
   {
      if (faces.size() == 0)
         return null;
      else if (faces.size() == 1)
         return faces.get(0).isInteriorPoint(pointToCheck) ? null : faces.get(0);

      for (int i = 0; i < faces.size(); i++)
      {
         tempVector.sub(pointToCheck, faces.get(i).getEdge(0).getOriginVertex().getPosition());
         double dotProduct = tempVector.dot(faces.get(i).getFaceNormal());
         if (dotProduct >= epsilon || faces.get(i).getNumberOfEdges() < 3)
         {
            return faces.get(i);
         }
      }
      return null;
   }

   public boolean isInteriorPoint(Point3DReadOnly pointToCheck, double epsilon)
   {
      return isInteriorPointInternal(pointToCheck, epsilon) == null;
   }

   @Override
   public Point3D getSupportingVertex(Vector3D supportDirection)
   {
      T bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      T vertexCandidate = bestVertex;
      while (true)
      {
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            tempVector.set(bestVertex.getAssociatedEdge(i).getDestinationVertex());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
               maxDotProduct = dotProduct;
            }
         }
         if (bestVertex == vertexCandidate)
            return (Point3D) bestVertex.getPosition();
         else
            bestVertex = vertexCandidate;
      }
   }

   // TODO Hacking this for the new collision detector. #FIXME fix this and the related interfaces and all that depends on those interfaces so that there dont need to be two versions of the getSupportinVertex function
   public T getSupportingVertexHack(Vector3DReadOnly supportDirection)
   {
      T bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      T vertexCandidate = bestVertex;
      while (true)
      {
         for (int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            tempVector.set(bestVertex.getAssociatedEdge(i).getDestinationVertex());
            double dotProduct = supportDirection.dot(tempVector);
            if (dotProduct > maxDotProduct)
            {
               vertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
               maxDotProduct = dotProduct;
            }
         }
         if (bestVertex == vertexCandidate)
            return bestVertex;
         else
            bestVertex = vertexCandidate;
      }
   }

   public String toString()
   {
      String string = "\n\nNumber of faces: " + faces.size();
      for (int i = 0; i < faces.size(); i++)
      {
         string = string + "\n" + faces.get(i).toString();
      }
      return string;
   }

   @Override
   public boolean epsilonEquals(V other, double epsilon)
   {
      // TODO imlepment this
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
      }
   }

   @Override
   public void setToZero()
   {
      // This should also set all the edges and vertices to zero assuming all relationships are intact
      for (int i = 0; i < faces.size(); i++)
      {
         faces.get(i).setToZero();
      }
   }

   @Override
   public void set(V other)
   {
      setFaces(other.getFaces());
   }

   private void setFaces(List<U> faces)
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

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      if (isInteriorPoint(point, Epsilons.ONE_TRILLIONTH))
      {
         return -getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
      else
      {
         return getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
   }

   public U getFaceContainingPointClosestTo(Point3DReadOnly point)
   {
      if (faces.size() == 0)
         throw new RuntimeException();
      else if (faces.size() == 1)
      {
         return faces.get(0);
      }

      unmarkAllFaces();
      U currentBestFace = faces.get(0);
      U faceUnderConsideration = currentBestFace;
      double maxDotProduct = faceUnderConsideration.getFaceVisibilityProduct(point);
      faceUnderConsideration.mark();

      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < currentBestFace.getNumberOfEdges(); j++)
         {
            if (currentBestFace.getNeighbouringFace(j) != null && currentBestFace.getNeighbouringFace(j).isNotMarked())
            {
               double dotProduct = currentBestFace.getNeighbouringFace(j).getFaceVisibilityProduct(point);
               if (dotProduct > maxDotProduct)
               {
                  maxDotProduct = dotProduct;
                  faceUnderConsideration = currentBestFace.getNeighbouringFace(j);
               }
               currentBestFace.getNeighbouringFace(j).mark();
            }
         }
         if (faceUnderConsideration == currentBestFace)
            break;
         else
            currentBestFace = faceUnderConsideration;
      }
      return currentBestFace;
   }

   private void updateCentroid()
   {
      updateVertices();
      centroid.setToZero();
      for (int i = 0; i < vertices.size(); i++)
         centroid.add(vertices.get(i));
      centroid.scale(1.0 / vertices.size());
   }

   public Point3DReadOnly getCentroid()
   {
      updateCentroid();
      return centroid;
   }

   protected abstract PolytopeVertexProvider<T, S, U, Q> getVertexProvider();
   protected abstract ConvexPolytopeFaceProvider<T, S, U, Q> getConvexFaceProvider();
   
   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      getFaceContainingPointClosestTo(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      getFaceContainingPointClosestTo(point).getSupportVectorJacobianTo(point, jacobianToPack);
   }

   @Override
   public SimplexBasics<Q> getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getSmallestSimplexMemberReference(point);
   }
}
