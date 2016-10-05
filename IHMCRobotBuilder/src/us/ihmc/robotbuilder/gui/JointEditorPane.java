package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.layout.BorderPane;
import org.controlsfx.control.PropertySheet;
import us.ihmc.robotbuilder.gui.editors.EditorFactory;
import us.ihmc.robotbuilder.gui.editors.ImmutableBeanEditor;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

/**
 *
 */
public class JointEditorPane extends BorderPane
{

   public JointEditorPane(JointDescription jointDescription)
   {
      Property<JointDescription> jointDescription1 = new SimpleObjectProperty<>();
      jointDescription1.setValue(jointDescription);

      PropertySheet propertySheet = new ImmutableBeanEditor<>(jointDescription, new EditorFactory(), false).getEditor();
      setCenter(propertySheet);
   }
}
