package frc.lib.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class NavX extends AHRS
{	
    private AHRS ahrs = new AHRS();

	public NavX(Port spi_port_id) 
	{
        super(spi_port_id);
	}

    public double getHeading(){ return ahrs.getYaw() + 180; }

    public double getRotationRate(){ return ahrs.getRawGyroZ(); }
}