package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import javax.swing.text.html.parser.DocumentParser;

import com.sun.media.jfxmedia.events.NewFrameEvent;

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

/**
 * A convex polytope is a collection of faces that describe it 
 * 
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ConvexPolytope implements GeometryObject<ConvexPolytope>, SupportingVertexHolder
{
   private final ArrayList<PolytopeVertex> vertices = new ArrayList<>();
   private final ArrayList<PolytopeHalfEdge> edges = new ArrayList<>();
   private final ArrayList<ConvexPolytopeFace> faces = new ArrayList<>();
   /**
    * Bounding box for the polytope
    */
   private boolean boundingBoxNeedsUpdating = false;
   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
                                                               Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final ArrayList<ConvexPolytopeFace> visibleFaces = new ArrayList<>();
   private final ArrayList<ConvexPolytopeFace> silhouetteFaces = new ArrayList<>();
   private final ArrayList<ConvexPolytopeFace> nonSilhouetteFaces = new ArrayList<>();
   private final ArrayList<ConvexPolytopeFace> onFaceList = new ArrayList<>();
   private final ArrayList<PolytopeHalfEdge> visibleSilhouetteList = new ArrayList<>();
   private List<PolytopeHalfEdge> visibleFaceEdgeList1 = new ArrayList<>();
   private List<PolytopeHalfEdge> visibleFaceEdgeList2 = new ArrayList<>();

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
      // Polyhedron formula for quick calc
      return getNumberOfEdges() - getNumberOfFaces() + 2;
   }

   public List<PolytopeVertex> getVertices()
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
         for (int j = 0; j < faces.get(j).getNumberOfEdges(); j++)
         {
            if (!faces.get(i).getEdge(j).getOriginVertex().isAnyFaceMarked())
            {
               vertices.add(faces.get(i).getEdge(j).getOriginVertex());
            }
         }
         faces.get(i).mark();
      }
   }

   public PolytopeVertex getVertex(int index)
   {
      updateVertices();
      return vertices.get(index);
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

   public List<ConvexPolytopeFace> getFaces()
   {
      return faces;
   }

   public ConvexPolytopeFace getFace(int index)
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

   public void addVertices(List<PolytopeVertex> vertices, double epsilon)
   {
      for (int i = 0; i < vertices.size(); i++)
         addVertex(vertices.get(i), epsilon);
   }

   public void addVertex(double epsilon, double... coordinates)
   {
      addVertex(new PolytopeVertex(coordinates[0], coordinates[1], coordinates[2]), epsilon);
   }

   public void addVertex(double x, double y, double z, double epsilon)
   {
      addVertex(new PolytopeVertex(x, y, z), epsilon);
   }

   public void addVertex(Point3D vertexToAdd, double epsilon)
   {
      addVertex(new PolytopeVertex(vertexToAdd), epsilon);
   }

   /**
    * Adds a polytope vertex to the current polytope. 
    * In case needed faces are removed and recreated. This will result in garbage. Fix if possible
    * @param vertexToAdd
    * @param epsilon
    * @return
    */
   public void addVertex(PolytopeVertex vertexToAdd, double epsilon)
   {
      PrintTools.debug(" ******* Adding vertex ***** ");
      if (faces.size() == 0)
      {
         // Polytope is empty. Creating face and adding the vertex
         ConvexPolytopeFace newFace = new ConvexPolytopeFace();
         newFace.addVertex(vertexToAdd);
         faces.add(newFace);
         boundingBoxNeedsUpdating = true;
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
            if (faces.get(0).isFaceVisible(vertexToAdd, epsilon))
               faces.get(0).reverseFaceNormal();

            visibleSilhouetteList.clear();
            PolytopeHalfEdge halfEdge = faces.get(0).getEdge(0);
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

      getVisibleFaces(visibleFaces, vertexToAdd, epsilon);
//      for (int i = 0; i < visibleFaces.size(); i++)
//         PrintTools.debug("Visible Face: " + visibleFaces.get(i).toString());
      if (visibleFaces.isEmpty())
         return;
      else
         PrintTools.debug("Got visible faces");
      getFacesWhichPointIsOn(vertexToAdd, onFaceList, Epsilons.ONE_BILLIONTH);
      getSilhouetteFaces(silhouetteFaces, nonSilhouetteFaces, visibleFaces);
      PrintTools.debug("Got silhouette faces");
      removeFaces(nonSilhouetteFaces);
      PrintTools.debug(onFaceList.size() + " ");
      // onFaceList.size() > 0 ? onFaceList.get(0).getFirstVisibleEdge(vertexToAdd).getTwinHalfEdge()      : 
      PolytopeHalfEdge firstHalfEdgeForSilhouette = getSeedEdgeForSilhouetteCalculation(silhouetteFaces);
      PrintTools.debug("Silhouette seed: " + firstHalfEdgeForSilhouette == null ? "null" : firstHalfEdgeForSilhouette.toString());
      getVisibleSilhouetteUsingSeed(visibleSilhouetteList, firstHalfEdgeForSilhouette, silhouetteFaces);
      PrintTools.debug("Got silhouette");
      removeFaces(silhouetteFaces);
      createFacesFromVisibleSilhouetteAndOnFaceList(visibleSilhouetteList, onFaceList, vertexToAdd);
      PrintTools.debug(" ******** Done adding vertex ********");
      boundingBoxNeedsUpdating = true;
   }

   private double getTripleProduct(Vector3DReadOnly v1, Vector3DReadOnly v2, Vector3DReadOnly v3)
   {
      tempVector.cross(v1, v2);
      return tempVector.dot(v3);
   }

   private PolytopeHalfEdge getCommonEdge(ConvexPolytopeFace face1, ConvexPolytopeFace face2)
   {
      PolytopeHalfEdge edgeCandidate = null;
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

   public void getSilhouetteFaces(List<ConvexPolytopeFace> silhouetteFacesToPack, List<ConvexPolytopeFace> nonSilhouetteFacesToPack,
                                  List<ConvexPolytopeFace> visibleFaceList)
   {
      if (silhouetteFacesToPack != null)
         silhouetteFacesToPack.clear();
      if (nonSilhouetteFacesToPack != null)
         nonSilhouetteFacesToPack.clear();
      for (int i = 0; i < visibleFaceList.size(); i++)
      {
         ConvexPolytopeFace candidateFace = visibleFaceList.get(i);

         boolean allNeighbouringFacesVisible = true;
         for (int j = 0; j < candidateFace.getNumberOfEdges(); j++)
            allNeighbouringFacesVisible &= visibleFaceList.contains(candidateFace.getNeighbouringFace(j));

         if (allNeighbouringFacesVisible && nonSilhouetteFacesToPack != null)
            nonSilhouetteFacesToPack.add(candidateFace);
         else if (silhouetteFacesToPack != null)
            silhouetteFacesToPack.add(candidateFace);
      }
   }

   public void getVisibleSilhouette(Point3DReadOnly vertex, List<PolytopeHalfEdge> visibleSilhouetteToPack, double epsilon)
   {
      getVisibleFaces(visibleFaces, vertex, epsilon);
      if (visibleFaces.isEmpty())
         return;
      getSilhouetteFaces(silhouetteFaces, null, visibleFaces);
      getVisibleSilhouetteUsingSeed(visibleSilhouetteToPack, getSeedEdgeForSilhouetteCalculation(silhouetteFaces), silhouetteFaces);
   }

   public void getVisibleSilhouetteUsingSeed(List<PolytopeHalfEdge> visibleSilhouetteToPack, PolytopeHalfEdge seedHalfEdge,
                                             List<ConvexPolytopeFace> silhouetteFaceList)
   {
      int iteration = 0;
      PolytopeHalfEdge halfEdgeUnderConsideration = seedHalfEdge;
      visibleSilhouetteToPack.clear();
      PrintTools.debug("Seed edge: " + seedHalfEdge.toString());
//      for(int i = 0; i < silhouetteFaceList.size(); i++)
//      {
//         PrintTools.debug(silhouetteFaceList.get(i).toString());
//      }
      while (true)
      {
         iteration++;
         visibleSilhouetteToPack.add(halfEdgeUnderConsideration.getTwinHalfEdge());
         //PrintTools.debug("Adding: " + halfEdgeUnderConsideration.toString());
         PolytopeVertex destinationVertex = halfEdgeUnderConsideration.getDestinationVertex();
         for (int i = 0; i < destinationVertex.getNumberOfAssociatedEdges(); i++)
         {
            //PrintTools.debug("Edge: " + destinationVertex.getAssociatedEdge(i).toString() + " "
            //      + silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getFace()) + " asdasd "
            //            + silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge().getFace()) );
            if (silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getFace())
                  && !silhouetteFaceList.contains(destinationVertex.getAssociatedEdge(i).getTwinHalfEdge().getFace()))
            {
               halfEdgeUnderConsideration = destinationVertex.getAssociatedEdge(i);
               break;
            }
         }
         if (halfEdgeUnderConsideration == seedHalfEdge || iteration > 10)
            break;
      }
   }

   public PolytopeHalfEdge getSeedEdgeForSilhouetteCalculation(List<ConvexPolytopeFace> silhouetteFaceList)
   {
      ConvexPolytopeFace seedFaceCandidate = silhouetteFaceList.get(0);
      PolytopeHalfEdge seedEdgeCandidate = seedFaceCandidate.getEdge(0);
      for (int i = 0; i < seedFaceCandidate.getNumberOfEdges(); i++)
      {
         if (silhouetteFaceList.contains(seedEdgeCandidate.getTwinHalfEdge().getFace())
               && !silhouetteFaceList.contains(seedEdgeCandidate.getNextHalfEdge().getTwinHalfEdge().getFace()))
            break;
         seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
      }
      seedEdgeCandidate = seedEdgeCandidate.getNextHalfEdge();
      return seedEdgeCandidate;
   }

   //   private void createFacesFromVisibleSilhouette(PolytopeVertex vertexToAdd)
   //   {
   //      ConvexPolytopeFace firstNewFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(0));
   //      twinEdges(visibleSilhouetteList.get(0), firstNewFace.getEdge(0));
   //      for (int i = 1; i < visibleSilhouetteList.size(); i++)
   //      {
   //         ConvexPolytopeFace newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, visibleSilhouetteList.get(i));
   //         twinEdges(visibleSilhouetteList.get(i - 1).getTwinHalfEdge().getNextHalfEdge(), newFace.getEdge(0).getPreviousHalfEdge());
   //      }
   //      twinEdges(visibleSilhouetteList.get(visibleSilhouetteList.size() - 1).getTwinHalfEdge().getNextHalfEdge(), firstNewFace.getEdge(0).getPreviousHalfEdge());
   //   }

   private void createFacesFromVisibleSilhouetteAndOnFaceList(List<PolytopeHalfEdge> silhouetteEdges, List<ConvexPolytopeFace> onFaceList, PolytopeVertex vertexToAdd)
   {
      //for(int i = 0; i < silhouetteEdges.size(); i++)
      //PrintTools.debug("Sil: " + silhouetteEdges.get(i));
      //PrintTools.debug(silhouetteEdges.get(0).getFace().toString());
      PolytopeHalfEdge previousLeadingEdge = null, trailingEdge = null;
      if(onFaceList.contains(silhouetteEdges.get(0).getFace()))
      {
         previousLeadingEdge = silhouetteEdges.get(0).getFace().getFirstVisibleEdge(vertexToAdd);
         silhouetteEdges.get(0).getFace().addVertex(vertexToAdd);
         trailingEdge = previousLeadingEdge.getNextHalfEdge();
      }
      else
      {
         ConvexPolytopeFace firstFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(0));
         previousLeadingEdge = firstFace.getEdge(0).getNextHalfEdge();
         trailingEdge = firstFace.getEdge(0).getPreviousHalfEdge();
         //PrintTools.debug("This happened");
      }
      //PrintTools.debug("PrevLeadEdge: "  + ((previousLeadingEdge == null )? "null" : previousLeadingEdge.toString()));
      //PrintTools.debug("TrailEdge: "  + ((trailingEdge == null) ? "null" : trailingEdge.toString() ));
      for (int i = 1; i < silhouetteEdges.size(); i++)
      {
         //PrintTools.debug("Previous leading: " + previousLeadingEdge.toString() + " Visible : " + visibleSilhouetteList.get(i).toString());
         if(onFaceList.contains(silhouetteEdges.get(i).getFace()))
         {
            PolytopeHalfEdge tempEdge = silhouetteEdges.get(i).getFace().getFirstVisibleEdge(vertexToAdd);
            silhouetteEdges.get(i).getFace().addVertex(vertexToAdd);
            twinEdges(previousLeadingEdge, tempEdge.getNextHalfEdge());
            previousLeadingEdge = tempEdge;
         }
         else
         {
            ConvexPolytopeFace newFace = createFaceFromTwinEdgeAndVertex(vertexToAdd, silhouetteEdges.get(i));
            twinEdges(previousLeadingEdge, newFace.getEdge(0).getPreviousHalfEdge());
            previousLeadingEdge = newFace.getEdge(0).getNextHalfEdge();
         }
      }
      twinEdges(previousLeadingEdge, trailingEdge);
   }

   private void twinEdges(PolytopeHalfEdge halfEdge1, PolytopeHalfEdge halfEdge2)
   {
      halfEdge1.setTwinHalfEdge(halfEdge2);
      halfEdge2.setTwinHalfEdge(halfEdge1);
   }

   private ConvexPolytopeFace createFaceFromTwinEdgeAndVertex(PolytopeVertex vertex, PolytopeHalfEdge twinEdge)
   {
      ConvexPolytopeFace newFace = new ConvexPolytopeFace();
      faces.add(newFace);
      newFace.addVertex(twinEdge.getDestinationVertex());
      newFace.addVertex(twinEdge.getOriginVertex());
      newFace.addVertex(vertex);
      twinEdges(newFace.getEdge(0), twinEdge);
      return newFace;
   }

   private void removeFaces(List<ConvexPolytopeFace> facesToRemove)
   {
      for (int i = 0; i < facesToRemove.size(); i++)
      {
         removeFace(facesToRemove.get(i));
      }
   }

   public void getVisibleFaces(List<ConvexPolytopeFace> faceReferencesToPack, Point3DReadOnly vertexUnderConsideration, double epsilon)
   {
      faceReferencesToPack.clear();
      for (int i = 0; i < faces.size(); i++)
      {
         if (faces.get(i).isFaceVisible(vertexUnderConsideration, epsilon))
         {
            faceReferencesToPack.add(faces.get(i));
         }
      }
   }

   public void getFacesWhichPointIsOn(Point3DBasics vertexUnderConsideration, List<ConvexPolytopeFace> faceReferenceToPack, double epsilon)
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

   public void removeFace(ConvexPolytopeFace faceToRemove)
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

   private ConvexPolytopeFace isInteriorPointInternal(Point3D pointToCheck, double epsilon)
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
      PolytopeVertex bestVertex = faces.get(0).getEdge(0).getOriginVertex();
      tempVector.set(bestVertex);
      double maxDotProduct = supportDirection.dot(tempVector);
      PolytopeVertex vertexCandidate = bestVertex;
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
            return bestVertex.getPosition();
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
   public void set(ConvexPolytope other)
   {
      setFaces(other.getFaces());
   }

   private void setFaces(List<ConvexPolytopeFace> faces)
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
