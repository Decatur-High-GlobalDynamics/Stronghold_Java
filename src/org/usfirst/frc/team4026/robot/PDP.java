package org.usfirst.frc.team4026.robot;

public class PDP extends edu.wpi.first.wpilibj.PowerDistributionPanel {
	PDP(){
		super();
	}
	
	double current(int x){
		return this.getCurrent(x);
	}
	double voltage(){
		return this.getVoltage();
	}
	double totalCurrent(){
		return this.getTotalCurrent();
	}
	double totalPower(){
		return this.getTotalPower();
	}
	double totalEnergy(){
		return this.getTotalEnergy();
	}
}
