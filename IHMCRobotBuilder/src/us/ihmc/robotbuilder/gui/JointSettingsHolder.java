package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.control.ScrollPane;
import us.ihmc.robotbuilder.gui.editors.RecursiveBeanEditor;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotbuilder.util.TreeFocus;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.util.Optional;

import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 *
 */
public class JointSettingsHolder extends ScrollPane
{
   private Property<JointDescription> editorProperty = noCycle(new SimpleObjectProperty<>());
   private TreeFocus<Tree<JointDescription>> currentlyEditedNode = null;
   private final RecursiveBeanEditor<JointDescription> editor = new RecursiveBeanEditor<>(editorProperty);

   public JointSettingsHolder()
   {
      setFitToWidth(true);
   }

   public void setFocusProperty(Property<Optional<TreeFocus<Tree<JointDescription>>>> focusProperty)
   {
      focusProperty.addListener((observable, oldValue, newValue) -> {
         setContent(null);
         newValue.ifPresent(newFocus -> {
            currentlyEditedNode = newFocus;
            editorProperty.setValue(newFocus.getFocusedNode().getValue());
         });
      });
      editorProperty.addListener((observable, oldValue, newValue) -> {
         TreeFocus<Tree<JointDescription>> editedTree = currentlyEditedNode.replace(currentlyEditedNode.getFocusedNode().withValue(newValue));
         focusProperty.setValue(Optional.ofNullable(editedTree));
      });

      editorProperty.addListener((observable, oldValue, newValue) -> setContent(editor.getEditor()));
   }


}
