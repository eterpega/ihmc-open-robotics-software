
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for limit complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="limit">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="lower" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="upper" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="effort" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="velocity" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "limit", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Limit {

    @XmlAttribute(name = "lower")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double lower;
    @XmlAttribute(name = "upper")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double upper;
    @XmlAttribute(name = "effort")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double effort;
    @XmlAttribute(name = "velocity")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double velocity;

    /**
     * Gets the value of the lower property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getLower() {
        if (lower == null) {
            return  0.0D;
        } else {
            return lower;
        }
    }

    /**
     * Sets the value of the lower property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setLower(Double value) {
        this.lower = value;
    }

    /**
     * Gets the value of the upper property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getUpper() {
        if (upper == null) {
            return  0.0D;
        } else {
            return upper;
        }
    }

    /**
     * Sets the value of the upper property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setUpper(Double value) {
        this.upper = value;
    }

    /**
     * Gets the value of the effort property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getEffort() {
        if (effort == null) {
            return  0.0D;
        } else {
            return effort;
        }
    }

    /**
     * Sets the value of the effort property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setEffort(Double value) {
        this.effort = value;
    }

    /**
     * Gets the value of the velocity property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getVelocity() {
        if (velocity == null) {
            return  0.0D;
        } else {
            return velocity;
        }
    }

    /**
     * Sets the value of the velocity property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setVelocity(Double value) {
        this.velocity = value;
    }

}
