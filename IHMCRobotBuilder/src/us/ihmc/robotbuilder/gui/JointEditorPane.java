package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.layout.BorderPane;
import org.controlsfx.control.PropertySheet;
import org.controlsfx.control.PropertySheet.Item;
import org.controlsfx.property.BeanProperty;
import us.ihmc.robotbuilder.gui.editors.EditorFactory;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

import java.beans.IntrospectionException;
import java.beans.PropertyDescriptor;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;

/**
 *
 */
public class JointEditorPane extends BorderPane
{

   private Property<JointDescription> jointDescription = new SimpleObjectProperty<>();

   public JointEditorPane(JointDescription jointDescription)
   {
      this.jointDescription.setValue(jointDescription);

      PropertySheet propertySheet = new PropertySheet();
      propertySheet.setPropertyEditorFactory(new EditorFactory());
      propertySheet.getItems().addAll(getProperties(jointDescription.toModifiable()));
      setCenter(propertySheet);

      //buildUI();
   }

   private static ObservableList<Item> getProperties(final Object bean) {
      ObservableList<Item> list = FXCollections.observableArrayList();
      list.addAll(getPropertyDescriptions(bean).stream()
                                               .map(p -> new BeanProperty(bean, p))
                                               .collect(Collectors.toList()));
      return list;
   }

   private static List<PropertyDescriptor> getPropertyDescriptions(final Object bean)
   {
      final String getPrefix = "get";
      return Arrays.stream(bean.getClass().getMethods())
            .filter(method -> isPublic(method.getModifiers()))
            .filter(method -> !isStatic(method.getModifiers()))
            .filter(method -> method.getName().startsWith(getPrefix))
            .map(getMethod -> {
               try
               {
                  String propertyName = getMethod.getName().substring(getPrefix.length());
                  Method setMethod = findSetter(bean.getClass(), propertyName);
                  if (setMethod == null)
                     return null;
                  return new PropertyDescriptor(propertyName, getMethod, setMethod);
               }
               catch (IntrospectionException e)
               {
                  e.printStackTrace();
                  return null;
               }
            })
            .filter(property -> property != null)
            .collect(Collectors.toList());
   }

   private static Method findSetter(Class<?> clazz, String propertyName)
   {
      return Arrays.stream(clazz.getMethods())
            .filter(method -> method.getName().startsWith("set"))
            .filter(method -> method.getName().contains(propertyName))
            .findFirst().orElse(null);
   }
}
