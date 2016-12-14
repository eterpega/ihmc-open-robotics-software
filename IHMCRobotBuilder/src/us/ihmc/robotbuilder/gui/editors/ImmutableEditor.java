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
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.gui.Editor;
import us.ihmc.robotbuilder.gui.ModifiableProperty;

import java.util.Objects;
import java.util.function.Supplier;

import static us.ihmc.robotbuilder.util.ReflectionHelpers.ImmutableReflectionProperty;
import static us.ihmc.robotbuilder.util.ReflectionHelpers.propertiesForBeanWithImmutableSetters;
import static us.ihmc.robotbuilder.util.ReflectionHelpers.propertiesForImmutableBeanWithBuilderConstructor;
import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.NoCycleProperty.noCycle;

/**
 *
 */
public class ImmutableEditor<T> extends Editor<T>
{
   private final GridPane editor = new GridPane();
   private final Editor.Factory editorFactory;
   private List<ModifiableBeanProperty> properties = List.empty();
   private T currentlyEditedBean = null;

   public ImmutableEditor(Property<T> valueProperty, Editor.Factory editorFactory)
   {
      super(noCycle(valueProperty));
      this.editorFactory = editorFactory;

      functional(valueProperty())
            .filter(Objects::nonNull)
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
            .map(property -> Tuple.of(property, editorFactory.create(property.getValueType(), property.getGenericTypes(), property.value())))
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

   @SuppressWarnings("unchecked")
   private List<ModifiableBeanProperty> getProperties(final Class<?> beanClass)
   {
      java.util.List<ImmutableReflectionProperty<T>> properties = propertiesForBeanWithImmutableSetters((Class<T>) beanClass);
      if (properties.isEmpty())
         properties = propertiesForImmutableBeanWithBuilderConstructor((Class<T>)beanClass);

      return List.ofAll(properties)
            .map(property -> new ModifiableBeanProperty(beanClass, property));
   }

   private class ModifiableBeanProperty implements ModifiableProperty<Object>
   {
      private final ImmutableReflectionProperty<T> property;
      private final Property<Object> valueProperty = new SimpleObjectProperty<Object>() {
         @Override public String getName()
         {
            return property.getName();
         }

         @Override public Object getBean()
         {
            return beanProperty().getValue();
         }
      };
      private final ChangeListener<Object> propertyChangeListener;
      private final ChangeListener<T> valueChangeListener;

      ModifiableBeanProperty(Class<?> beanType, ImmutableReflectionProperty<T> property)
      {
         this.property = property;

         propertyChangeListener = (observable, oldValue, newValue) -> {
            if (beanProperty().getValue() == null || !beanProperty().getValue().getClass().equals(beanType))
               return;

            currentlyEditedBean = property.getSetter().withValue(beanProperty().getValue(), newValue)
                                          .getOrElseThrow((Supplier<RuntimeException>)RuntimeException::new);
            beanProperty().setValue(currentlyEditedBean);
         };

         valueChangeListener = (observable, oldValue, newBean) ->
         {
            if (!newBean.getClass().equals(beanType))
               return;

            valueProperty.setValue(property.getGetter().getValue(newBean)
                                           .getOrElseThrow((Supplier<RuntimeException>)RuntimeException::new));
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
         return property.getName();
      }

      @SuppressWarnings("unchecked") @Override
      public Class<Object> getValueType()
      {
         return (Class<Object>)property.getType();
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

      java.util.List<Class<?>> getGenericTypes()
      {
         return property.getGenericTypes();
      }
   }
}
