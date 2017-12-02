package us.ihmc.avatar.collisionAvoidance;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FrameConvexPolytopeVisualizer
{
   private final int numberOfVizEdges;
   private final int numberOfVizVertices;
   private final YoVariableRegistry registry;
   private final YoGraphicsListRegistry graphicsListRegistry;
   private ArrayList<YoGraphicPosition> polytopeVerticesViz;
   private ArrayList<YoGraphicLineSegment> polytopeEdgesViz;
   private YoGraphicPosition position;
   private final ConvexPolytopeReadOnly[] polytopes;
   private final Color[] polytopeColors;
   private int numberOfPolytopes = 0;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public FrameConvexPolytopeVisualizer(int maxNumberOfPolytopes, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.registry = registry;
      this.graphicsListRegistry = graphicsListRegistry;
      this.polytopes = new ConvexPolytopeReadOnly[maxNumberOfPolytopes];
      this.polytopeColors = new Color[maxNumberOfPolytopes];
      this.numberOfVizEdges = maxNumberOfPolytopes * 150;
      this.numberOfVizVertices = maxNumberOfPolytopes * 50;
      polytopeVerticesViz = new ArrayList<>(numberOfVizVertices);
      polytopeEdgesViz = new ArrayList<>(numberOfVizEdges);
      createPolytopeVisualizationElements();
   }

   public void addPolytope(ConvexPolytopeReadOnly polytopeToAdd)
   {
      polytopes[numberOfPolytopes] = polytopeToAdd;
      polytopeColors[numberOfPolytopes] = getNextColor();
      numberOfPolytopes++;
      updatePolytopeVisualization(polytopes);
   }

   private Color getNextColor()
   {
      double numberOfDivisionsPerColor = Math.pow(polytopes.length, 1.0 / 3.0);
      double r = MathTools.clamp((numberOfPolytopes % numberOfDivisionsPerColor) / (numberOfDivisionsPerColor - 1), 0.0, 1.0);
      double g = MathTools.clamp(((numberOfPolytopes / numberOfDivisionsPerColor) % numberOfDivisionsPerColor) / (numberOfDivisionsPerColor - 1), 0.0, 1.0);
      double b = MathTools.clamp(((numberOfPolytopes / numberOfDivisionsPerColor / numberOfDivisionsPerColor) % numberOfDivisionsPerColor)
            / (numberOfDivisionsPerColor - 1), 0.0, 1.0);
      return new Color((float) r, (float) g, (float) b);
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

   private YoGraphicLineSegment xVector;
   private YoGraphicLineSegment yVector;
   private YoGraphicLineSegment zVector;

   public void createPolytopeVisualizationElements()
   {
      xVector = new YoGraphicLineSegment("xAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      xVector.setDrawArrowhead(true);
      yVector = new YoGraphicLineSegment("yAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      yVector.setDrawArrowhead(true);
      zVector = new YoGraphicLineSegment("zAxis", "Viz", worldFrame, new YoAppearanceRGBColor(Color.PINK, 0.0), registry);
      zVector.setDrawArrowhead(true);
      graphicsListRegistry.registerYoGraphic("Axis", xVector);
      graphicsListRegistry.registerYoGraphic("Axis", yVector);
      graphicsListRegistry.registerYoGraphic("Axis", zVector);

      polytopeEdgesViz.clear();
      for (int i = 0; i < numberOfVizEdges; i++)
      {
         YoGraphicLineSegment edge = new YoGraphicLineSegment("PolytopeEdge" + i, "Viz", worldFrame, new YoAppearanceRGBColor(Color.GRAY, 0.5), registry);
         edge.setDrawArrowhead(false);
         edge.setToNaN();
         polytopeEdgesViz.add(edge);
      }
      graphicsListRegistry.registerYoGraphics("PolytopeEdges", polytopeEdgesViz);

      for (int i = 0; i < numberOfVizVertices; i++)
      {
         YoGraphicPosition point = new YoGraphicPosition("PolytopeVertex" + i, "Viz", registry, 0.001, new YoAppearanceRGBColor(Color.GRAY, 0.0));
         point.setPositionToNaN();
         polytopeVerticesViz.add(point);
      }
      graphicsListRegistry.registerYoGraphics("PolytopeVertices", polytopeVerticesViz);

      position = new YoGraphicPosition("PositionForVisibleEdges", "Viz", registry, 0.1, new YoAppearanceRGBColor(Color.GRAY, 0.5));
      position.setPositionToNaN();
      graphicsListRegistry.registerYoGraphic("VisualPoint", position);
   }

   public void updatePolytopeVisualization(ConvexPolytopeReadOnly... polytopes)
   {
      int edgeIndex = 0;
      int vertexIndex = 0;
      for (int j = 0; j < numberOfPolytopes && polytopes[j] != null; j++)
      {
         Color color = polytopeColors[j];
         List<? extends PolytopeHalfEdgeReadOnly> edges = polytopes[j].getEdges();
         List<? extends PolytopeVertexReadOnly> vertices = polytopes[j].getVertices();
         int i = 0;
         for (i = 0; i < edges.size(); i++)
         {
            polytopeEdgesViz.get(edgeIndex).setStartAndEnd(edges.get(i).getOriginVertex().getPosition(), edges.get(i).getDestinationVertex().getPosition());
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
   }

   public void updateColor(FrameConvexPolytope polytopeToChange, Color newColor)
   {
      for (int i = 0; i < numberOfPolytopes; i++)
         if (polytopes[i] == polytopeToChange)
            polytopeColors[i] = newColor;
      updatePolytopeVisualization(polytopes);
   }

   public void removePolytope(ConvexPolytopeReadOnly polytope)
   {
      int i = 0;
      for (; i < numberOfPolytopes; i++)
      {
         if (polytopes[i] == polytope)
         {
            polytopes[i] = null;
            numberOfPolytopes--;
            break;
         }
      }
      for (; i < polytopes.length - 1; i++)
         polytopes[i] = polytopes[i + 1];
      polytopes[polytopes.length - 1] = null;
      update();
   }
}