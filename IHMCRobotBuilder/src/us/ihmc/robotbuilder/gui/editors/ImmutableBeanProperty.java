package us.ihmc.robotbuilder.gui.editors;

import impl.org.controlsfx.i18n.Localization;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ObservableValue;
import org.controlsfx.control.PropertySheet;
import org.controlsfx.control.PropertySheet.Item;
import org.jetbrains.annotations.NotNull;
import us.ihmc.robotbuilder.util.FunctionalObservableValue;

import java.beans.FeatureDescriptor;
import java.beans.PropertyDescriptor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Optional;

/**
 * A convenience class for creating a {@link Item} for use in the
 * {@link PropertySheet} control based on a property belonging to an immutable bean.
 *
 * @see Item
 * @see PropertySheet
 * @see PropertyDescriptor
 * @see org.controlsfx.property.BeanProperty
 */
@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
public class ImmutableBeanProperty<BeanType> implements Item
{

   /**
    * Unique identifier to provide a custom category label within
    * {@link Item#getCategory()}.
    *
    * How to use it: with a PropertyDescriptor, provide the custom category
    * through a a named attribute
    * {@link FeatureDescriptor#setValue(String, Object)}.
    *
    * <pre>
    * final PropertyDescriptor propertyDescriptor = new PropertyDescriptor("yourProperty", YourBean.class);
    * propertyDescriptor.setDisplayName("Your Display Name");
    * propertyDescriptor.setShortDescription("Your explanation about this property.");
    * // then provide a custom category
    * propertyDescriptor.setValue(BeanProperty.CATEGORY_LABEL_KEY, "Your custom category");
    * </pre>
    */
   private final Property<BeanType> beanProperty;
   private final FunctionalObservableValue<Object> valueProperty;
   private final String propertyName;
   private final String propertyCategory;
   private final String description;
   private final Method readMethod;
   private final Optional<Method> writeMethod;
   private final Class<?> propertyType;
   private boolean editable = true;

   public ImmutableBeanProperty(final BeanType bean, String propertyName, String propertyCategory, String description, @NotNull Method readMethod, Optional<Method> writeMethod, Class<?> propertyType)
   {
      this.beanProperty = new SimpleObjectProperty<>(bean);
      this.propertyName = propertyName;
      this.propertyCategory = propertyCategory;
      this.description = description;
      this.readMethod = readMethod;
      this.writeMethod = writeMethod;
      this.propertyType = propertyType;
      this.valueProperty = FunctionalObservableValue.of(beanProperty).map(x -> this.getValue());
      this.setEditable(writeMethod.isPresent());
   }

   /** {@inheritDoc} */
   @Override public String getName()
   {
      return propertyName;
   }

   /** {@inheritDoc} */
   @Override public String getDescription()
   {
      return description;
   }

   /** {@inheritDoc} */
   @Override public Class<?> getType()
   {
      return propertyType;
   }

   /** {@inheritDoc} */
   @Override public Object getValue()
   {
      try
      {
         return this.readMethod.invoke(this.beanProperty.getValue());
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         e.printStackTrace();
         return null;
      }
   }

   /** {@inheritDoc} */
   @Override public void setValue(final Object value)
   {
      this.writeMethod.ifPresent(writeMethod ->
      {
         try
         {
            @SuppressWarnings("unchecked")
            BeanType result = (BeanType)writeMethod.invoke(this.beanProperty.getValue(), value);
            this.beanProperty.setValue(result);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });
   }

   /** {@inheritDoc} */
   @Override public String getCategory()
   {
      // fall back to default behavior if there is no category provided.
      if (propertyCategory == null || propertyCategory.isEmpty())
      {
         return Localization.localize(Localization.asKey("bean.property.category.basic"));
      }
      return propertyCategory;
   }

   /**
    * @return The object passed in to the constructor of the BeanProperty.
    */
   public BeanType getBean()
   {
      return this.beanProperty.getValue();
   }

   /** {@inheritDoc} */
   @Override public boolean isEditable()
   {
      return this.editable;
   }

   /**
    * @param editable Whether this property should be editable in the PropertySheet.
    */
   public void setEditable(final boolean editable)
   {
      this.editable = editable;
   }

   /** {@inheritDoc} */
   @Override public Optional<ObservableValue<?>> getObservableValue()
   {
      return Optional.of(valueProperty);
   }

   public Property<BeanType> beanProperty()
   {
      return beanProperty;
   }
}
