package us.ihmc.robotbuilder.gui;

import javafx.scene.Node;
import javafx.scene.control.Label;

/**
 * Label with default font set to FontAwesome. Can be used
 * as a graphics node for icons.
 */
public class FontAwesomeLabel extends Label
{
   /**
    * Creates an empty label
    */
   public FontAwesomeLabel()
   {
      init();
   }

   /**
    * Creates Label with supplied text.

    * @param text null text is treated as the empty string
    */
   public FontAwesomeLabel(String text)
   {
      super(text);
      init();
   }

   /**
    * Creates a Label with the supplied text and graphic.

    * @param text null text is treated as the empty string
    * @param graphic a null graphic is acceptable
    */
   public FontAwesomeLabel(String text, Node graphic)
   {
      super(text, graphic);
      init();
   }

   private void init()
   {
      setStyle("-fx-font-family: \"FontAwesome\";");
   }
}
