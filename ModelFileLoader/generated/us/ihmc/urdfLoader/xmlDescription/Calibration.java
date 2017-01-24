
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for calibration complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="calibration">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="reference_position" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="rising" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="falling" type="{http://www.w3.org/2001/XMLSchema}double" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "calibration", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Calibration {

    @XmlAttribute(name = "reference_position")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double referencePosition;
    @XmlAttribute(name = "rising")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double rising;
    @XmlAttribute(name = "falling")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double falling;

    /**
     * Gets the value of the referencePosition property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Double getReferencePosition() {
        return referencePosition;
    }

    /**
     * Sets the value of the referencePosition property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setReferencePosition(Double value) {
        this.referencePosition = value;
    }

    /**
     * Gets the value of the rising property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Double getRising() {
        return rising;
    }

    /**
     * Sets the value of the rising property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setRising(Double value) {
        this.rising = value;
    }

    /**
     * Gets the value of the falling property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Double getFalling() {
        return falling;
    }

    /**
     * Sets the value of the falling property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setFalling(Double value) {
        this.falling = value;
    }

}
