package us.ihmc.robotbuilder.gui.editors;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.scene.Node;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javaslang.Tuple;
import javaslang.collection.List;
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.ModifiableProperty;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;
import static us.ihmc.robotbuilder.util.FunctionalObservableValue.functional;
import static us.ihmc.robotbuilder.util.NoCycleProperty.noCycle;

/**
 *
 */
public class ImmutableEditor<T> extends Editor<T>
{
   private final GridPane editor = new GridPane();
   private Class<?> currentBeanClass = null;
   private final Editor.Factory editorFactory;
   private List<ModifiableBeanProperty> properties = List.empty();

   public ImmutableEditor(Property<T> valueProperty, Editor.Factory editorFactory)
   {
      super(noCycle(valueProperty));
      this.editorFactory = editorFactory;

      functional(valueProperty())
            .filter(newValue -> newValue != null)
            .consume(this::updateUIFromBean);
   }

   private void updateUIFromBean(@NotNull T bean)
   {
      if (currentBeanClass == bean.getClass())
         return;

      currentBeanClass = bean.getClass();

      properties.forEach(ModifiableBeanProperty::unregisterObservers);
      editor.getChildren().clear();

      properties = getProperties(bean);
      //noinspection OptionalGetWithoutIsPresent
      properties
            .map(property -> Tuple.of(property, editorFactory.create(property.getValueType(), property.value(), property.getName())))
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
                        editor.add(propertyEditor.getEditor(), 1, index);
                     });
   }

   @Override public Node getEditor()
   {
      return editor;
   }

   private T getBean()
   {
      return valueProperty().getValue();
   }

   private List<ModifiableBeanProperty> getProperties(final T bean)
   {
      final String getPrefix = "get";
      final Set<String> usedProperties = new HashSet<>();
      return List.ofAll(Arrays.asList(bean.getClass().getMethods()))
                   .filter(method -> isPublic(method.getModifiers()))
                   .filter(method -> !isStatic(method.getModifiers()))
                   .filter(method -> method.getName().startsWith(getPrefix))
                   .filter(method -> method.getParameterCount() == 0)
                   .filter(method -> !usedProperties.contains(method.getName()))
                   .map(getMethod -> {
                      usedProperties.add(getMethod.getName());
                      String propertyName = getMethod.getName().substring(getPrefix.length());
                      Method setMethod = findSetter(bean.getClass(), propertyName, getMethod.getReturnType());
                      if (setMethod != null)
                         //noinspection unchecked
                         return new ModifiableBeanProperty(propertyName, bean.getClass(), (Class)getMethod.getReturnType(), getMethod, setMethod);
                      return null;
                   })
                   .filter(item -> item != null);
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
      private final Property<Object> valueProperty = new SimpleObjectProperty<>();
      private final ChangeListener<Object> propertyChangeListener;
      private final ChangeListener<T> valueChangeListener;

      ModifiableBeanProperty(String name, Class<?> beanType, Class<Object> valueType, Method getter, Method setter)
      {
         this.name = name;
         this.valueType = valueType;

         propertyChangeListener = (observable, oldValue, newValue) -> {
            if (valueProperty().getValue() == null || !valueProperty().getValue().getClass().equals(beanType))
               return;
            try
            {
               //noinspection unchecked
               valueProperty().setValue((T)setter.invoke(getBean(), newValue));
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

         ImmutableEditor.this.valueProperty().addListener(valueChangeListener);

         if (ImmutableEditor.this.valueProperty().getValue() != null)
            valueChangeListener.changed(ImmutableEditor.this.valueProperty(), null, ImmutableEditor.this.valueProperty().getValue());
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
