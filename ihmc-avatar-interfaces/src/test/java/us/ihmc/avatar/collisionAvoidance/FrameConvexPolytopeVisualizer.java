package us.ihmc.avatar.collisionAvoidance;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor.Axis;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// This is a copy of Polytope visualizer
public class FrameConvexPolytopeVisualizer
{
   private static final int numberOfVizEdges = 5000;
   private static final int numberOfVizVertices = 1000;
   private static final int numberOfVizVisiblePlanes = 50;
   private static final int numberOfVizVisibleSilhouetteEdges = 50;
   private final YoVariableRegistry registry = new YoVariableRegistry("PolytopeVisualizer");
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private SimulationConstructionSet scs;
   private ArrayList<YoGraphicPosition> polytopeVerticesViz;
   private ArrayList<YoGraphicLineSegment> polytopeEdgesViz;
   private ArrayList<YoGraphicLineSegment> visibleSilhouetteViz;
   private ArrayList<YoGraphicPosition> visibleFaceViz;
   private YoGraphicPosition position;
   private YoDouble yoTime;
   private final FrameConvexPolytope[] polytopes;
   private final Color[] polytopeColors;
   private int numberOfPolytopes = 0;
   private YoGraphicLineSegment collisionVector;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private boolean keepSCSUp = false;

   public FrameConvexPolytopeVisualizer(int maxNumberOfPolytopes)
   {
      this(maxNumberOfPolytopes, false);
   }
   public FrameConvexPolytopeVisualizer(int maxNumberOfPolytopes, boolean keepSCSUp)
   {
      this.keepSCSUp = keepSCSUp;
      polytopeVerticesViz = new ArrayList<>(numberOfVizVertices);
      polytopeEdgesViz = new ArrayList<>(numberOfVizEdges);
      visibleSilhouetteViz = new ArrayList<>(numberOfVizVisibleSilhouetteEdges);
      visibleFaceViz = new ArrayList<>(numberOfVizVisiblePlanes);
      polytopes = new FrameConvexPolytope[maxNumberOfPolytopes];
      polytopeColors = new Color[maxNumberOfPolytopes];
      createPolytopeVisualizationElements();
      setupSCS();
   }

   public void addPolytope(FrameConvexPolytope polytopeToAdd, Color color)
   {
      polytopes[numberOfPolytopes] = polytopeToAdd;
      polytopeColors[numberOfPolytopes] = color;
      numberOfPolytopes++;
   }

   public void update()
   {
      updatePolytopeVisualization(polytopes);
      if(keepSCSUp)
         ThreadTools.sleepForever();
   }

   private void setupSCS()
   {
      Robot robot = new Robot(getClass().getSimpleName() + "Robot");
      yoTime = robot.getYoTime();
      robot.addYoVariableRegistry(registry);
      robot.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      scs = new SimulationConstructionSet(robot, parameters);
      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(coordinateSystem);
      scs.setGroundVisible(false);
      scs.setDT(1.0, 1);
      scs.startOnAThread();
   }

   public void tickSCS()
   {
      yoTime.add(1.0);
      scs.tickAndUpdate();
   }

   private Point3D tempPoint1 = new Point3D();
   private Point3D tempPoint2 = new Point3D();

   public void showCollisionVector(Vector3D collisionVector)
   {
      tempPoint2.set(tempPoint1);
      tempPoint2.add(collisionVector);
      this.collisionVector.setStartAndEnd(tempPoint1, tempPoint2);
      tickSCS();
   }

   public void createPolytopeVisualizationElements()
   {
      collisionVector = new YoGraphicLineSegment("CollisionVector", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      collisionVector.setDrawArrowhead(true);
      collisionVector.setToNaN();
      graphicsListRegistry.registerYoGraphic("CollisionVector", collisionVector);

      visibleFaceViz.clear();
      for (int i = 0; i < numberOfVizVisiblePlanes; i++)
      {
         YoGraphicPosition polygon = new YoGraphicPosition("VisibleFaceCentroid" + i, "Viz", registry, 0.1, new YoAppearanceRGBColor(Color.BLUE, 0.25));
         polygon.setPositionToNaN();
         visibleFaceViz.add(polygon);
      }
      graphicsListRegistry.registerYoGraphics("VisibleFaces", visibleFaceViz);

      polytopeEdgesViz.clear();
      for (int i = 0; i < numberOfVizEdges; i++)
      {
         YoGraphicLineSegment edge = new YoGraphicLineSegment("PolytopeEdge" + i, "Viz", worldFrame, new YoAppearanceRGBColor(Color.BLACK, 0.5), registry);
         edge.setDrawArrowhead(true);
         edge.setToNaN();
         polytopeEdgesViz.add(edge);
      }
      graphicsListRegistry.registerYoGraphics("PolytopeEdges", polytopeEdgesViz);

      visibleSilhouetteViz.clear();
      for (int i = 0; i < numberOfVizVisibleSilhouetteEdges; i++)
      {
         YoGraphicLineSegment edge = new YoGraphicLineSegment("VisibleEdge" + i, "Viz", worldFrame, new YoAppearanceRGBColor(Color.GREEN, 0.0), registry);
         edge.setDrawArrowhead(true);
         edge.setToNaN();
         visibleSilhouetteViz.add(edge);
      }
      graphicsListRegistry.registerYoGraphics("VisibleEdges", visibleSilhouetteViz);

      for (int i = 0; i < numberOfVizVertices; i++)
      {
         YoGraphicPosition point = new YoGraphicPosition("PolytopeVertex" + i, "Viz", registry, 0.01, new YoAppearanceRGBColor(Color.BLACK, 0.5));
         point.setPositionToNaN();
         polytopeVerticesViz.add(point);
      }
      graphicsListRegistry.registerYoGraphics("PolytopeVertices", polytopeVerticesViz);

      position = new YoGraphicPosition("PositionForVisibleEdges", "Viz", registry, 0.1, new YoAppearanceRGBColor(Color.RED, 0.5));
      position.setPositionToNaN();
      graphicsListRegistry.registerYoGraphic("VisualPoint", position);
   }

   public void updatePolytopeVisualization(FrameConvexPolytope... polytopes)
   {
      int edgeIndex = 0;
      int vertexIndex = 0;
      for (int j = 0; j < numberOfPolytopes; j++)
      {
         Color color = polytopeColors[j];
         List<FramePolytopeHalfEdge> edges = polytopes[j].getEdges();
         List<FramePolytopeVertex> vertices = polytopes[j].getVertices();
         int i = 0;
         for (i = 0; i < edges.size(); i++)
         {
            polytopeEdgesViz.get(edgeIndex).setStartAndEnd(edges.get(i).getOriginVertex().getPosition().getPoint(),
                                                           edges.get(i).getDestinationVertex().getPosition().getPoint());
            polytopeEdgesViz.get(edgeIndex).getAppearance().getColor().set(color);
            edgeIndex++;
         }
         for (i = 0; i < vertices.size(); i++)
         {
            polytopeVerticesViz.get(vertexIndex).setPosition(vertices.get(i).getPosition());
            vertexIndex++;
         }
      }
      for (; edgeIndex < polytopeEdgesViz.size(); edgeIndex++)
         polytopeEdgesViz.get(edgeIndex).setToNaN();
      for (; vertexIndex < polytopeVerticesViz.size(); vertexIndex++)
         polytopeVerticesViz.get(vertexIndex).setPositionToNaN();
      tickSCS();
   }

   public void updateVisibleSilhouetteEdges(FrameConvexPolytope polytope, FramePolytopeVertex vertex)
   {
      List<FramePolytopeHalfEdge> visibleSilhouetteToPack = new ArrayList<>();
      polytope.getVisibleSilhouette(vertex, visibleSilhouetteToPack, Epsilons.ONE_TEN_THOUSANDTH);
      int index = 0;
      position.setPosition(vertex.getPosition());
      for (index = 0; index < visibleSilhouetteToPack.size(); index++)
      {
         visibleSilhouetteViz.get(index).setStartAndEnd(visibleSilhouetteToPack.get(index).getOriginVertex().getPosition().getPoint(),
                                                        visibleSilhouetteToPack.get(index).getDestinationVertex().getPosition().getPoint());
         tickSCS();
      }
      for (; index < visibleSilhouetteViz.size(); index++)
         visibleSilhouetteViz.get(index).setToNaN();
   }

   public void updateVisibleFaces(FrameConvexPolytope polytope, FramePolytopeVertex vertex)
   {
      List<FrameConvexPolytopeFace> visibleFaces = new ArrayList<>();
      polytope.getVisibleFaces(visibleFaces, vertex, Epsilons.ONE_BILLIONTH);
      position.setPosition(vertex.getPosition());
      int index = 0;
      for (index = 0; index < visibleFaces.size(); index++)
      {
         visibleFaceViz.get(index).setPosition(visibleFaces.get(index).getFaceCentroid());
         tickSCS();
      }
      for (; index < visibleFaceViz.size(); index++)
         visibleFaceViz.get(index).setPositionToNaN();
   }
   
   public void addVerticesForViz(ArrayList<Point3D> pointList)
   {
      int index;
      for(index = 0; index < pointList.size(); index++)
      {
         polytopeVerticesViz.get(index).setPosition(pointList.get(index));
      }
      for(; index < polytopeVerticesViz.size(); index++)
      {
         polytopeVerticesViz.get(index).setPositionToNaN();
      }
      
      tickSCS();
   }
   
   public static void main(String args[])
   {
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(1);
      //ArrayList<Point3D> pointList = ConvexPolytopeConstructor.getCollisionMeshPointsForCapsule(0.0, 0.0, 0.0, Axis.Z, 1, 0.2, 8);
      //viz.addVerticesForViz(pointList);
      FrameConvexPolytope capsule = ConvexPolytopeConstructor.getFrameCapsuleCollisionMesh(new FramePoint3D(), Axis.Z, 1, 0.5, 4);
      viz.addPolytope(capsule, Color.BLUE);
      viz.update();
   }

   private static void testRobotMeshProvider()
   {
      FrameConvexPolytopeVisualizer viz = new FrameConvexPolytopeVisualizer(1);
      RobotCollisionMeshProvider meshProvider = new RobotCollisionMeshProvider(10);
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBody rigidBody = new RigidBody("RigidBody", transform, worldFrame);
      ArrayList<CollisionMeshDescription> collisionMeshDescriptionList = new ArrayList<>();
      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.addCylinderReferencedAtCenter(0.5, 2);
      collisionMesh.addCubeReferencedAtCenter(1, 1, 1);
      collisionMeshDescriptionList.add(collisionMesh);
      FrameConvexPolytope frameConvexPolytope = meshProvider.createCollisionMesh(rigidBody, collisionMeshDescriptionList);
      viz.addPolytope(frameConvexPolytope, Color.CYAN);
      viz.update();
   }
}