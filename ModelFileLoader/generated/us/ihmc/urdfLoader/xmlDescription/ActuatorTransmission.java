
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for actuator_transmission complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="actuator_transmission">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="mechanicalReduction" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="name" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "actuator_transmission", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class ActuatorTransmission {

    @XmlAttribute(name = "mechanicalReduction", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double mechanicalReduction;
    @XmlAttribute(name = "name", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;

    /**
     * Gets the value of the mechanicalReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getMechanicalReduction() {
        return mechanicalReduction;
    }

    /**
     * Sets the value of the mechanicalReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMechanicalReduction(double value) {
        this.mechanicalReduction = value;
    }

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getName() {
        return name;
    }

    /**
     * Sets the value of the name property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setName(String value) {
        this.name = value;
    }

}
