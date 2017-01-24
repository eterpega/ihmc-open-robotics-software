
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for mimic complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="mimic">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="joint" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="multiplier" type="{http://www.w3.org/2001/XMLSchema}double" default="1" />
 *       &lt;attribute name="offset" type="{http://www.w3.org/2001/XMLSchema}double" default="0" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "mimic", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Mimic {

    @XmlAttribute(name = "joint", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String joint;
    @XmlAttribute(name = "multiplier")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double multiplier;
    @XmlAttribute(name = "offset")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Double offset;

    /**
     * Gets the value of the joint property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getJoint() {
        return joint;
    }

    /**
     * Sets the value of the joint property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setJoint(String value) {
        this.joint = value;
    }

    /**
     * Gets the value of the multiplier property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getMultiplier() {
        if (multiplier == null) {
            return  1.0D;
        } else {
            return multiplier;
        }
    }

    /**
     * Sets the value of the multiplier property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMultiplier(Double value) {
        this.multiplier = value;
    }

    /**
     * Gets the value of the offset property.
     * 
     * @return
     *     possible object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getOffset() {
        if (offset == null) {
            return  0.0D;
        } else {
            return offset;
        }
    }

    /**
     * Sets the value of the offset property.
     * 
     * @param value
     *     allowed object is
     *     {@link Double }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setOffset(Double value) {
        this.offset = value;
    }

}
