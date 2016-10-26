package us.ihmc.robotbuilder.gui.editors2;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
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
import java.util.concurrent.atomic.AtomicBoolean;

import static java.lang.reflect.Modifier.isPublic;
import static java.lang.reflect.Modifier.isStatic;

/**
 *
 */
public class ImmutableEditor<T> extends Editor<T>
{
   private final GridPane editor = new GridPane();
   private Class<?> currentBeanClass = null;
   private final Editor.Factory editorFactory;

   public ImmutableEditor(Property<T> valueProperty, Editor.Factory editorFactory)
   {
      super(valueProperty);
      this.editorFactory = editorFactory;

      valueProperty.addListener((observable, oldValue, newValue) ->
                                {
                                   if (newValue != null)
                                      updateUIFromBean(newValue);
                                });
   }

   private void updateUIFromBean(@NotNull T bean)
   {
      if (currentBeanClass == bean.getClass())
         return;

      currentBeanClass = bean.getClass();

      List<ModifiableBeanProperty> properties = getProperties(bean);
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
                         return new ModifiableBeanProperty(propertyName, (Class)getMethod.getReturnType(), getMethod, setMethod);
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

      ModifiableBeanProperty(String name, Class<Object> valueType, Method getter, Method setter)
      {
         this.name = name;
         this.valueType = valueType;

         AtomicBoolean ignoreEdits = new AtomicBoolean(false);
         valueProperty.addListener((observable, oldValue, newValue) ->
                                   {
                                      if (ignoreEdits.getAndSet(true))
                                         return;

                                      try
                                      {
                                         //noinspection unchecked
                                         valueProperty().setValue((T)setter.invoke(getBean(), newValue));
                                      }
                                      catch (Exception e)
                                      {
                                         throw new RuntimeException(e);
                                      } finally
                                      {
                                         ignoreEdits.getAndSet(false);
                                      }

                                   });

         ImmutableEditor.this.valueProperty().addListener((observable, oldValue, newValue) ->
                                                          {
                                                             if (ignoreEdits.getAndSet(true))
                                                                return;

                                                             try
                                                             {
                                                                valueProperty.setValue(getter.invoke(newValue));
                                                             }
                                                             catch (Exception e)
                                                             {
                                                                throw new RuntimeException(e);
                                                             } finally
                                                             {
                                                                ignoreEdits.set(false);
                                                             }
                                                          });
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
   }
}
