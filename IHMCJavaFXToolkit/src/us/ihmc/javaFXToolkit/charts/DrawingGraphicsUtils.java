package us.ihmc.javaFXToolkit.charts;

import javafx.scene.paint.Color;
import javafx.scene.shape.Line;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.StrokeType;

/**
 * Created by amoucheboeuf on 5/17/16.
 */
public class DrawingGraphicsUtils // TODO improve this class and make it more generic
{


   public static Line makeLine(Color color, double width)
   {
      Line line = new Line();
      line.setStroke(color);
      line.setStrokeWidth(width);
      return line;
   }


   public static Rectangle makeSelectionRectangle(Color fillColor, Color strokeColor, float strokeWidth)
   {
      Rectangle rectangle = new Rectangle(0, 0, 0, 0);
      rectangle.setFill(fillColor);
      rectangle.setOpacity(0.2);
      rectangle.setStroke(strokeColor);
      rectangle.setStrokeType(StrokeType.INSIDE);
      rectangle.setStrokeWidth(strokeWidth);
      rectangle.setMouseTransparent(true);

      return rectangle;
   }


}
