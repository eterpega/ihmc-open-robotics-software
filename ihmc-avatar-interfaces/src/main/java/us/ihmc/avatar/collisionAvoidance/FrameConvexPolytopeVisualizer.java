package us.ihmc.avatar.collisionAvoidance;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FrameConvexPolytopeVisualizer
{
   private final ArrayList<YoGraphicPosition> polytopeVerticesViz;
   private final ArrayList<YoGraphicLineSegment> polytopeEdgesViz;
   private final YoGraphicLineSegment xVector;
   private final YoGraphicLineSegment yVector;
   private final YoGraphicLineSegment zVector;

   private final ConvexPolytopeReadOnly[] polytopes;
   private final Color[] polytopeColors;
   private int numberOfPolytopes = 0;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public FrameConvexPolytopeVisualizer(int maxNumberOfPolytopes, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(maxNumberOfPolytopes, 150, 50, registry, graphicsListRegistry);
   }

   public FrameConvexPolytopeVisualizer(int maxNumberOfPolytopes, int maxNumberOfVerticesPerPolytope, int maxNumberOfEdgesPerPolytope,
                                        YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.polytopes = new ConvexPolytopeReadOnly[maxNumberOfPolytopes];
      this.polytopeColors = new Color[maxNumberOfPolytopes];

      int maxNumberOfEdges = maxNumberOfPolytopes * maxNumberOfEdgesPerPolytope;
      int maxNumberOfVertices = maxNumberOfPolytopes * maxNumberOfVerticesPerPolytope;

      polytopeVerticesViz = new ArrayList<>(maxNumberOfVertices);
      polytopeEdgesViz = new ArrayList<>(maxNumberOfEdges);

      String listName = "FrameConvexPolytopeVisualizer";

      xVector = new YoGraphicLineSegment("xAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      xVector.setDrawArrowhead(true);
      yVector = new YoGraphicLineSegment("yAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      yVector.setDrawArrowhead(true);
      zVector = new YoGraphicLineSegment("zAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      zVector.setDrawArrowhead(true);
      graphicsListRegistry.registerYoGraphic(listName, xVector);
      graphicsListRegistry.registerYoGraphic(listName, yVector);
      graphicsListRegistry.registerYoGraphic(listName, zVector);

      polytopeEdgesViz.clear();
      for (int i = 0; i < maxNumberOfEdges; i++)
      {
         YoGraphicLineSegment edge = new YoGraphicLineSegment("PolytopeEdge" + i, "Viz", worldFrame, new YoAppearanceRGBColor(Color.GRAY, 0.5), registry);
         edge.setDrawArrowhead(false);
         edge.setToNaN();
         polytopeEdgesViz.add(edge);
      }
      graphicsListRegistry.registerYoGraphics(listName, polytopeEdgesViz);

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         YoGraphicPosition point = new YoGraphicPosition("PolytopeVertex" + i, "Viz", registry, 0.001, new YoAppearanceRGBColor(Color.GRAY, 0.0));
         point.setPositionToNaN();
         polytopeVerticesViz.add(point);
      }
      graphicsListRegistry.registerYoGraphics(listName, polytopeVerticesViz);
   }

   public void clear()
   {
      numberOfPolytopes = 0;
      Arrays.fill(polytopes, null);
      update();
   }

   public void addPolytope(ConvexPolytopeReadOnly polytopeToAdd, Color color)
   {
      polytopes[numberOfPolytopes] = polytopeToAdd;
      polytopeColors[numberOfPolytopes] = color;
      numberOfPolytopes++;
   }

   public void update()
   {
      updatePolytopeVisualization(polytopes);
   }

   private void updatePolytopeVisualization(ConvexPolytopeReadOnly[] polytopes)
   {
      int edgeIndex = 0;
      int vertexIndex = 0;

      for (int j = 0; j < numberOfPolytopes && polytopes[j] != null; j++)
      {
         Color color = polytopeColors[j];
         List<? extends PolytopeHalfEdgeReadOnly> edges = polytopes[j].getEdges();
         List<? extends PolytopeVertexReadOnly> vertices = polytopes[j].getVertices();

         for (int i = 0; i < edges.size(); i++)
         {
            polytopeEdgesViz.get(edgeIndex).setStartAndEnd(edges.get(i).getOriginVertex().getPosition(), edges.get(i).getDestinationVertex().getPosition());
            polytopeEdgesViz.get(edgeIndex).getAppearance().getColor().set(color);
            edgeIndex++;
         }
         for (int i = 0; i < vertices.size(); i++)
         {
            polytopeVerticesViz.get(vertexIndex).setPosition(vertices.get(i).getPosition());
            vertexIndex++;
         }
      }

      for (; edgeIndex < polytopeEdgesViz.size(); edgeIndex++)
      {
         polytopeEdgesViz.get(edgeIndex).hide();
      }

      for (; vertexIndex < polytopeVerticesViz.size(); vertexIndex++)
      {
         polytopeVerticesViz.get(vertexIndex).setPositionToNaN();
      }
   }
}