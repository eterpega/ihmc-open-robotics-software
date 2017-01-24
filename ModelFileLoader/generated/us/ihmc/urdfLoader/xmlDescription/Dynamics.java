
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for dynamics complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="dynamics">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="damping" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="friction" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "dynamics", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Dynamics {

    @XmlAttribute(name = "damping")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double damping;
    @XmlAttribute(name = "friction")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double friction;

    /**
     * Gets the value of the damping property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getDamping() {
        if (damping == null) {
            return  0.0D;
        } else {
            return damping;
        }
    }

    /**
     * Sets the value of the damping property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setDamping(Double value) {
        this.damping = value;
    }

    /**
     * Gets the value of the friction property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getFriction() {
        if (friction == null) {
            return  0.0D;
        } else {
            return friction;
        }
    }

    /**
     * Sets the value of the friction property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setFriction(Double value) {
        this.friction = value;
    }

}
