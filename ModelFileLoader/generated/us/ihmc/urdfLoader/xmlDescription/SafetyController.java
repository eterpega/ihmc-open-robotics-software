
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for safety_controller complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="safety_controller">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="soft_lower_limit" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="soft_upper_limit" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="k_position" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *       &lt;attribute name="k_velocity" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "safety_controller", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class SafetyController {

    @XmlAttribute(name = "soft_lower_limit")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double softLowerLimit;
    @XmlAttribute(name = "soft_upper_limit")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double softUpperLimit;
    @XmlAttribute(name = "k_position")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double kPosition;
    @XmlAttribute(name = "k_velocity", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double kVelocity;

    /**
     * Gets the value of the softLowerLimit property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getSoftLowerLimit() {
        if (softLowerLimit == null) {
            return  0.0D;
        } else {
            return softLowerLimit;
        }
    }

    /**
     * Sets the value of the softLowerLimit property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setSoftLowerLimit(Double value) {
        this.softLowerLimit = value;
    }

    /**
     * Gets the value of the softUpperLimit property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getSoftUpperLimit() {
        if (softUpperLimit == null) {
            return  0.0D;
        } else {
            return softUpperLimit;
        }
    }

    /**
     * Sets the value of the softUpperLimit property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setSoftUpperLimit(Double value) {
        this.softUpperLimit = value;
    }

    /**
     * Gets the value of the kPosition property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getKPosition() {
        if (kPosition == null) {
            return  0.0D;
        } else {
            return kPosition;
        }
    }

    /**
     * Sets the value of the kPosition property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setKPosition(Double value) {
        this.kPosition = value;
    }

    /**
     * Gets the value of the kVelocity property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getKVelocity() {
        return kVelocity;
    }

    /**
     * Sets the value of the kVelocity property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setKVelocity(double value) {
        this.kVelocity = value;
    }

}
