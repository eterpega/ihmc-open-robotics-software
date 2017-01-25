
package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for mass complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="mass">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="value" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "mass", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class URDFMass {

    @XmlAttribute(name = "value")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double value;

    /**
     * Gets the value of the value property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getValue() {
        if (value == null) {
            return  0.0D;
        } else {
            return value;
        }
    }

    /**
     * Sets the value of the value property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setValue(Double value) {
        this.value = value;
    }

}
