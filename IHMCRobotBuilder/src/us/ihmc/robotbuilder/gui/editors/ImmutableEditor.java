package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.geometry.Insets;
import javafx.geometry.VPos;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javaslang.Tuple;
import javaslang.collection.List;
import org.controlsfx.control.spreadsheet.Grid;
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.ModifiableProperty;

import java.lang.reflect.Method;
import java.util.Arrays;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;
import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 *
 */
public class ImmutableEditor<T> extends Editor<T>
{
   private static final String GET_PREFIX = "get";
   private final GridPane editor = new GridPane();
   private final Editor.Factory editorFactory;
   private List<ModifiableBeanProperty> properties = List.empty();
   private T currentlyEditedBean = null;

   public ImmutableEditor(Property<T> valueProperty, Editor.Factory editorFactory)
   {
      super(noCycle(valueProperty));
      this.editorFactory = editorFactory;

      functional(valueProperty())
            .filter(newValue -> newValue != null)
            .consume(this::updateUIFromBean);

      editor.setVgap(5);
      editor.setHgap(10);
   }

   private void updateUIFromBean(@NotNull T bean)
   {
      if (bean == currentlyEditedBean)
         return;

      currentlyEditedBean = bean;
      properties.forEach(ModifiableBeanProperty::unregisterObservers);
      editor.getChildren().clear();

      properties = getProperties(bean.getClass());
      //noinspection OptionalGetWithoutIsPresent
      properties
            .map(property -> Tuple.of(property, editorFactory.create(property.getValueType(), property.value())))
            .filter(propertyAndEditor -> propertyAndEditor._2.isPresent())
            .map(propertyAndEditor -> Tuple.of(propertyAndEditor._1, propertyAndEditor._2.get()))
            .zipWithIndex()
            .forEach(indexed ->
                     {
                        int index = (int) (long) indexed._2;
                        ModifiableBeanProperty property = indexed._1._1;
                        Editor<?> propertyEditor = indexed._1._2;

                        Label label = new Label(property.getName());
                        editor.add(label, 0, index);
                        label.setPadding(new Insets(5, 0, 0, 0));
                        GridPane.setValignment(label, VPos.TOP);
                        editor.add(propertyEditor.getEditor(), 1, index);
                     });
   }

   @Override public Node getEditor()
   {
      return editor;
   }

   private List<ModifiableBeanProperty> getProperties(final Class<?> beanClass)
   {
      final String getPrefix = "get";
      return List.ofAll(Arrays.asList(beanClass.getMethods()))
                   .filter(ImmutableEditor::isGetter)
                   .distinctBy(Method::getName)
                   .map(getMethod -> {
                      String propertyName = getMethod.getName().substring(getPrefix.length());
                      Method setMethod = findSetter(beanClass, propertyName, getMethod.getReturnType());
                      if (setMethod != null)
                         //noinspection unchecked
                         return new ModifiableBeanProperty(propertyName, beanClass, (Class)getMethod.getReturnType(), getMethod, setMethod);
                      return null;
                   })
                   .filter(item -> item != null);
   }

   private static boolean isGetter(Method method)
   {
      return isPublic(method.getModifiers()) && !isStatic(method.getModifiers()) &&
            method.getParameterCount() == 0 && method.getName().startsWith(GET_PREFIX);
   }

   private static Method findSetter(Class<?> clazz, String propertyName, Class<?> propertyType)
   {
      return Arrays.stream(clazz.getMethods())
                   .filter(method -> method.getName().startsWith("with") || method.getName().startsWith("set"))
                   .filter(method -> method.getName().contains(propertyName))
                   .filter(method -> clazz.isAssignableFrom(method.getReturnType()))
                   .filter(method -> method.getParameterCount() == 1 && method.getParameterTypes()[0].isAssignableFrom(propertyType))
                   .findFirst().orElse(null);
   }



   private class ModifiableBeanProperty implements ModifiableProperty<Object>
   {
      private final String name;
      private final Class<Object> valueType;
      private final Property<Object> valueProperty = new SimpleObjectProperty<Object>() {
         @Override public String getName()
         {
            return name;
         }

         @Override public Object getBean()
         {
            return beanProperty().getValue();
         }
      };
      private final ChangeListener<Object> propertyChangeListener;
      private final ChangeListener<T> valueChangeListener;

      ModifiableBeanProperty(String name, Class<?> beanType, Class<Object> valueType, Method getter, Method setter)
      {
         this.name = name;
         this.valueType = valueType;

         propertyChangeListener = (observable, oldValue, newValue) -> {
            if (beanProperty().getValue() == null || !beanProperty().getValue().getClass().equals(beanType))
               return;
            try
            {
               //noinspection unchecked
               T newBean = (T) setter.invoke(beanProperty().getValue(), newValue);
               currentlyEditedBean = newBean;
               beanProperty().setValue(newBean);
            }
            catch (Exception e)
            {
               throw new RuntimeException(e);
            }

         };

         valueChangeListener = (observable, oldValue, newValue) ->
         {
            if (!newValue.getClass().equals(beanType))
               return;
            try
            {
               valueProperty.setValue(getter.invoke(newValue));
            }
            catch (Exception e)
            {
               throw new RuntimeException(e);
            }
         };

         valueProperty.addListener(propertyChangeListener);

         beanProperty().addListener(valueChangeListener);

         if (beanProperty().getValue() != null)
            valueChangeListener.changed(beanProperty(), null, beanProperty().getValue());
      }

      private Property<T> beanProperty()
      {
         return ImmutableEditor.this.valueProperty();
      }

      @Override public String getName()
      {
         return name;
      }

      @Override public Class<Object> getValueType()
      {
         return valueType;
      }

      @Override public Property<Object> value()
      {
         return valueProperty;
      }

      private void unregisterObservers()
      {
         valueProperty.removeListener(propertyChangeListener);
         ImmutableEditor.this.valueProperty().removeListener(valueChangeListener);
      }
   }
}
