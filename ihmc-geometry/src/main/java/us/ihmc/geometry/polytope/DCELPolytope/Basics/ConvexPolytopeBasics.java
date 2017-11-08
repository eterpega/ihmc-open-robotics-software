package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.rmi.dgc.Lease;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.PolytopeListener;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

public abstract class ConvexPolytopeBasics<V extends PolytopeVertexBasics<V, E, F>, E extends PolytopeHalfEdgeBasics<V, E, F>, F extends ConvexPolytopeFaceBasics<V, E, F>>
      implements ConvexPolytopeReadOnly, SimplexBasics, Clearable, Transformable, Settable<ConvexPolytopeReadOnly>
{
   private final ArrayList<V> vertices = new ArrayList<>();
   private final ArrayList<E> edges = new ArrayList<>();
   private final ArrayList<F> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<F> visibleFaces = new ArrayList<>();
   private final ArrayList<F> silhouetteFaces = new ArrayList<>();
   private final ArrayList<F> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<F> onFaceList = new ArrayList<>();
   private final ArrayList<E> visibleSilhouetteList = new ArrayList<>();

   private Vector3D tempVector = new Vector3D();
   private Point3D centroid = new Point3D();
   private final PolytopeListener listener;

   public ConvexPolytopeBasics()
   {
      listener = null;
   }

   public ConvexPolytopeBasics(PolytopeListener listener)
   {
      this.listener = listener;
      this.listener.attachPolytope(this);
   }

   public ConvexPolytopeBasics(ConvexPolytopeReadOnly polytope)
   {
      set(polytope);
      boundingBoxNeedsUpdating = true;
      listener = null;
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

   public List<V> getVertices()
   {
      updateVertices();
      return vertices;
   }

   public void getVertices(List<Point3D> verticesToPack)
   {
      updateVertices();
      for (int i = 0; i < vertices.size(); i++)
         verticesToPack.get(i).set(vertices.get(i));
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

   public V getVertex(int index)
   {
      updateVertices();
      return vertices.get(index);
   }

   public int getNumberOfEdges()
   {
      updateEdges();
      return edges.size() / 2;
   }

   public List<E> getEdges()
   {
      updateEdges();
      return edges;
   }

   private void updateEdges()
   {
      edges.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         List<E> faceEdgeList = faces.get(i).getEdgeList();
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

   public List<F> getFaces()
   {
      return faces;
   }

   public F getFace(int index)
   {
      return faces.get(index);
   }

   public Vector3DReadOnly getFaceNormalAt(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getFaceNormal();
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

   public void addVertices(double epsilon, List<Point3D> vertices)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertices(List<V> vertices, double epsilon)
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
   public void addVertex(V vertexToAdd, double epsilon)
   {
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         F newFace = getConvexFaceProvider().getFace();
         newFace.addVertex(vertexToAdd, epsilon);
         faces.add(newFace);
         boundingBoxNeedsUpdating = true;
         updateListener();
         return;
      }
      else if (faces.size() == 1)
      {
         if (faces.get(0).isPointInFacePlane(vertexToAdd, epsilon))
         {
            if (!faces.get(0).isInteriorPoint(vertexToAdd, epsilon))
               faces.get(0).addVertex(vertexToAdd, epsilon);
            updateListener();
            return;
         }
         else
         {
            if (faces.get(0).isFaceVisible(vertexToAdd, epsilon))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            E halfEdge = faces.get(0).getEdge(0);
            if (listener != null)
               listener.udpateVisibleEdgeSeed(halfEdge);
            for (int i = 0; i < faces.get(0).getNumberOfEdges(); i++)
            {
               visibleSilhouetteList.add(halfEdge);
               halfEdge = halfEdge.getPreviousHalfEdge();
            }
            if (listener != null)
               listener.updateVisibleSilhouette(visibleSilhouetteList);
            onFaceList.clear();
            createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);
         }
         boundingBoxNeedsUpdating = true;
         updateListener();
         return;
      }
      F visibleFaceSeed = getVisibleFaces(visibleFaces, vertexToAdd, epsilon);
      if (visibleFaces.isEmpty())
      {
         updateListener();
         return;
      }
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, epsilon);
      //PrintTools.debug("Visible faces: " + visibleFaces.size() + ", On Faces: " + onFaceList.size());
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      E firstHalfEdgeForSilhouette = null;
      if(listener != null)
      {
         listener.updateOnFaceList(onFaceList);
         listener.updateVisibleFaceList(Arrays.asList(visibleFaceSeed));
      }
      if (onFaceList.size() > 0)
      {
         if(checkIsInteriorPointOf(onFaceList, vertexToAdd, epsilon))
            return;
         E firstVisibleEdge = getFirstVisibleEdgeFromOnFaceList(onFaceList, visibleFaces); //onFaceList.get(0).getFirstVisibleEdge(vertexToAdd);
         if(firstVisibleEdge == null)
            return;
         firstHalfEdgeForSilhouette = firstVisibleEdge.getTwinHalfEdge();
      }
      else
      {
         firstHalfEdgeForSilhouette = getSeedEdgeForSilhouetteCalculation(visibleFaces, visibleFaceSeed);
      }
      if (listener != null)
         listener.udpateVisibleEdgeSeed(firstHalfEdgeForSilhouette);
      if (firstHalfEdgeForSilhouette == null)
      {
         PrintTools.debug("Seed edge was null, aborting. On faces: " + onFaceList.size() + ", visible: " + visibleFaces.size());
         return;
      }
      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, visibleFaces);
      if (listener != null)
         listener.updateVisibleSilhouette(visibleSilhouetteList);
      if (visibleSilhouetteList.isEmpty())
      {
         PrintTools.debug("Empty visible silhouette ");
         updateListener();
         return;
      }
      removeFaces(nonSilhouetteFaces);
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd, epsilon);
      boundingBoxNeedsUpdating = true;
      updateListener();
   }

   private boolean checkIsInteriorPointOf(ArrayList<F> onFaceList, Point3DReadOnly vertexToAdd, double epsilon)
   {
      for(int i = 0; i < onFaceList.size(); i++)
      {
         if(onFaceList.get(i).getFirstVisibleEdge(vertexToAdd) == null)
            return true;
      }
      return false;
   }

   private E getFirstVisibleEdgeFromOnFaceList(ArrayList<F> onFaceList, ArrayList<F> visibleFaces)
   {
      F firstFace = onFaceList.get(0);
      E edgeUnderConsideration = firstFace.getEdge(0);
      for (int i = 0; i < firstFace.getNumberOfEdges(); i++)
      {
         if (!visibleFaces.contains(edgeUnderConsideration.getNextHalfEdge().getTwinHalfEdge().getFace())
               && visibleFaces.contains(edgeUnderConsideration.getTwinHalfEdge().getFace()))
            return edgeUnderConsideration;
         else
            edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
      }
      return null;
   }

   public void updateListener()
   {
      if (listener != null)
      {
         listener.updateAll();
      }
   }

   public void getSilhouetteFaces(List<F> silhouetteFacesToPack, List<F> nonSilhouetteFacesToPack, List<F> visibleFaceList)
   {
      if (silhouetteFacesToPack != null)
         silhouetteFacesToPack.clear();
      if (nonSilhouetteFacesToPack != null)
         nonSilhouetteFacesToPack.clear();
      for (int i = 0; i < visibleFaceList.size(); i++)
      {
         F candidateFace = visibleFaceList.get(i);

         boolean allNeighbouringFacesVisible = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesVisible &= visibleFaceList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesVisible && nonSilhouetteFacesToPack != null)
            nonSilhouetteFacesToPack.add(candidateFace);
         else if (silhouetteFacesToPack != null)
            silhouetteFacesToPack.add(candidateFace);
      }
   }

   public void getVisibleSilhouette(Point3DReadOnly vertex, List<E> visibleSilhouetteToPack, double epsilon)
   {
      F leastVisibleFace = getVisibleFaces(visibleFaces, vertex, epsilon);
      if (visibleFaces.isEmpty())
      {
         return;
      }
      getFacesWhichPointIsOn(vertex, onFaceList, epsilon);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      E firstHalfEdgeForSilhouette = onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertex).getTwinHalfEdge()
            : getSeedEdgeForSilhouetteCalculation(visibleFaces, leastVisibleFace);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, firstHalfEdgeForSilhouette, visibleFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<E> visibleSilhouetteToPack, E seedHalfEdge, List<F> silhouetteFaceList)
   {
      E halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      int numberOfEdges = getNumberOfEdges();
      int count;
      for (count = 0; count < numberOfEdges; count++)
      {
         if (halfEdgeUnderConsideration == null)
            PrintTools.debug("Half edge null " + faces.size());
         if (visibleSilhouetteToPack == null)
            PrintTools.debug("visible list null");
         if (halfEdgeUnderConsideration.getTwinHalfEdge() == null)
            PrintTools.debug("Twing half edge null");

         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinHalfEdge());
         V destinationVertex = halfEdgeUnderConsideration.getDestinationVertex();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            if(destinationVertex.getAssociatedEdge(i) == null)
               PrintTools.debug("Associated edge is null");
            if(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge() == null)
               PrintTools.debug("Associated edge twin is null\n" + toString());
            if(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge().getFace() == null)
               PrintTools.debug("Associated edge twin face is null");
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
      if (count == numberOfEdges && faces.size() > 1)
      {
         PrintTools.warn("Could not determine visible silhouette " + onFaceList.size() + ", " + silhouetteFaceList.size() + ", "
               + visibleSilhouetteToPack.size());
         PrintTools.warn("On face size: " + onFaceList.size());
         if(listener !=null)
            listener.updateOnFaceList(onFaceList);
         for (int i = 0; i < onFaceList.size(); i++)
         {
            PrintTools.debug(onFaceList.get(i).toString());
         }
         PrintTools.warn("Visible face size: " + visibleFaces.size());
         if(listener !=null)
            listener.updateVisibleFaceList(visibleFaces);
         for (int i = 0; i < visibleFaces.size(); i++)
         {
            PrintTools.debug(visibleFaces.get(i).toString());
         }
         if(listener !=null)
            listener.updateVisibleSilhouette(visibleSilhouetteToPack);
         visibleSilhouetteToPack.clear();
      }
   }

   public E getSeedEdgeForSilhouetteCalculation(List<F> visibleFaceList, F leastVisibleFace)
   {
      if (faces.size() == 1)
         return faces.get(0).getEdge(0);
      E seedEdge = null;
      E seedEdgeCandidate = leastVisibleFace.getEdge(0);
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

   private void createFacesFromVisibleSilhouetteAndOnFaceList(List<E> silhouetteEdges, List<F> onFaceList, V vertexToAdd, double epsilon)
   {
      //for(int i = 0; i < silhouetteEdges.size(); i++)
      //PrintTools.debug("Sil: " + silhouetteEdges.get(i));
      //PrintTools.debug(silhouetteEdges.get(0).getFace().toString());
      E previousLeadingEdge = null, trailingEdge = null;
      if (onFaceList.contains(silhouetteEdges.get(0).getFace()))
      {
         previousLeadingEdge = silhouetteEdges.get(0).getFace().getFirstVisibleEdge(vertexToAdd);
         if(previousLeadingEdge == null)
         {
            PrintTools.debug("vertex to add: " + vertexToAdd);
            PrintTools.debug("Face: " + silhouetteEdges.get(0).getFace().toString());
            PrintTools.debug("Polytope: " + toString());
         }
         silhouetteEdges.get(0).getFace().addVertex(vertexToAdd, epsilon);
         trailingEdge = previousLeadingEdge.getNextHalfEdge();
      }
      else
      {
         F firstFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(0), epsilon);
         previousLeadingEdge = firstFace.getEdge(0).getNextHalfEdge();
         trailingEdge = firstFace.getEdge(0).getPreviousHalfEdge();
      }
      //PrintTools.debug("PrevLeadEdge: "  + ((previousLeadingEdge == null )? "null" : previousLeadingEdge.toString()));
      //PrintTools.debug("TrailEdge: "  + ((trailingEdge == null) ? "null" : trailingEdge.toString() ));
      for (int i = 1; i < silhouetteEdges.size(); i++)
      {
         //PrintTools.debug("Previous leading: " + previousLeadingEdge.toString() + " Visible : " + visibleSilhouetteList.get(i).toString());
         if (onFaceList.contains(silhouetteEdges.get(i).getFace()))
         {
            F faceToExtend = silhouetteEdges.get(i).getFace();
            E tempEdge = faceToExtend.getFirstVisibleEdge(vertexToAdd);
            faceToExtend.addVertex(vertexToAdd, epsilon);
            if(tempEdge == null)
               PrintTools.debug("This again");
            if(tempEdge.getNextHalfEdge() == null)
               PrintTools.debug("WTF");
            twinEdges(previousLeadingEdge, tempEdge.getNextHalfEdge());
            previousLeadingEdge = tempEdge;
         }
         else
         {
            F newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(i), epsilon);
            twinEdges(previousLeadingEdge, newFace.getEdge(0).getPreviousHalfEdge());
            previousLeadingEdge = newFace.getEdge(0).getNextHalfEdge();
         }
      }
      twinEdges(previousLeadingEdge, trailingEdge);
   }

   private void twinEdges(E halfEdge1, E halfEdge2)
   {
      halfEdge1.setTwinHalfEdge(halfEdge2);
      halfEdge2.setTwinHalfEdge(halfEdge1);
   }

   private F createFaceFromTwinEdgeAndVertex(V vertex, E twinEdge, double epsilon)
   {
      F newFace = getConvexFaceProvider().getFace();
      faces.add(newFace);
      newFace.addVertex(twinEdge.getDestinationVertex(), epsilon);
      newFace.addVertex(twinEdge.getOriginVertex(), epsilon);
      newFace.addVertex(vertex, epsilon);
      twinEdges(newFace.getEdge(0), twinEdge);
      return newFace;
   }

   private void removeFaces(List<F> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
   }

   public F getVisibleFaces(List<F> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      F leastVisibleFace = null;
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

   public void getFacesWhichPointIsOn(Point3DReadOnly vertexUnderConsideration, List<F> faceReferenceToPack, double epsilon)
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

   public void removeFace(F faceToRemove)
   {
      for (int i = 0; i < faceToRemove.getNumberOfEdges(); i++)
      {
         E twinHalfEdge = faceToRemove.getEdge(i).getTwinHalfEdge();
         if (twinHalfEdge != null)
            twinHalfEdge.setTwinHalfEdge(null);
         faceToRemove.getEdge(i).clear();
      }
      faceToRemove.clearEdgeList();
      faces.remove(faceToRemove);
   }

   private F isInteriorPointInternal(Point3DReadOnly pointToCheck, double epsilon)
   {
      if (faces.size() == 0)
         return null;
      else if (faces.size() == 1)
         return faces.get(0).isInteriorPoint(pointToCheck, epsilon) ? null : faces.get(0);

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
      V bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      V vertexCandidate = bestVertex;

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
   @Override
   public V getSupportingVertexHack(Vector3DReadOnly supportDirection)
   {
      V bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      V vertexCandidate = bestVertex;
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
   public boolean epsilonEquals(ConvexPolytopeReadOnly other, double epsilon)
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
   public void set(ConvexPolytopeReadOnly other)
   {
      copyFaces(other.getFaces());
   }

   private void copyFaces(List<? extends ConvexPolytopeFaceReadOnly> faces)
   {
      //TODO implement this 
      throw new RuntimeException("Unimplemented feature");
   }

   public void clear()
   {
      vertices.clear();
      edges.clear();
      faces.clear();
      visibleFaces.clear();
      silhouetteFaces.clear();
      nonSilhouetteFaces.clear();
      onFaceList.clear();
      visibleSilhouetteList.clear();
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      if (isInteriorPoint(point, Epsilons.ONE_TRILLIONTH))
      {
         return getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
      else
      {
         return getFaceContainingPointClosestTo(point).getShortestDistanceTo(point);
      }
   }

   public F getFaceContainingPointClosestTo(Point3DReadOnly point)
   {
      if (faces.size() == 0)
         return null;
      else if (faces.size() == 1)
      {
         return faces.get(0);
      }

      unmarkAllFaces();
      F currentBestFace = faces.get(0);
      F faceUnderConsideration = currentBestFace;
      double minDistance = faceUnderConsideration.getShortestDistanceTo(point);
      faceUnderConsideration.mark();

      for (int i = 0; i < faces.size(); i++)
      {
         for (int j = 0; j < currentBestFace.getNumberOfEdges(); j++)
         {
            if (currentBestFace.getNeighbouringFace(j) != null && currentBestFace.getNeighbouringFace(j).isNotMarked())
            {
               double distance = currentBestFace.getNeighbouringFace(j).getShortestDistanceTo(point);
               if (distance < minDistance)
               {
                  minDistance = distance;
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

   protected abstract PolytopeVertexProvider<V, E, F> getVertexProvider();

   protected abstract ConvexPolytopeFaceProvider<V, E, F> getConvexFaceProvider();

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      getFaceContainingPointClosestTo(point).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return getFaceContainingPointClosestTo(point).getSmallestSimplexMemberReference(point);
   }
}
