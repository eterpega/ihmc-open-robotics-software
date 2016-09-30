package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.Group;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import org.controlsfx.control.PropertySheet;
import org.controlsfx.property.BeanPropertyUtils;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Stream;

/**
 *
 */
public class FloatingJointEditorPane extends BorderPane
{

   private Property<JointDescription> jointDescription = new SimpleObjectProperty<>();

   public FloatingJointEditorPane(JointDescription jointDescription)
   {
      this.jointDescription.setValue(jointDescription);

      PropertySheet propertySheet = new PropertySheet();
      propertySheet.getItems().addAll(BeanPropertyUtils.getProperties(jointDescription));
      setCenter(propertySheet);
      //buildUI();
   }

   /*private void buildUI() {
      AtomicInteger rowIndex = new AtomicInteger(0);
      getUIFields(jointDescription.getValue())
            .map(field -> Editor.editorForField(jointDescription.getValue(), field))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .forEach(editor -> add(editor, 0, rowIndex.getAndIncrement()));
   }

   private Stream<Field> getUIFields(Object object) {
      return Arrays.stream(object.getClass().getDeclaredFields())
            .filter(field -> Modifier.isFinal(field.getModifiers()) && !Modifier.isStatic(field.getModifiers()));
   }*/



}
