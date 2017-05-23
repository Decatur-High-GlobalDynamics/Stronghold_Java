package org.usfirst.frc.team4026.robot;

public class PDP extends edu.wpi.first.wpilibj.PowerDistributionPanel {
	PDP(){
		super();
	}
	
	double current(int x){
		return super.getCurrent(x);
	}
	double voltage(){
		return super.getVoltage();
	}
	double totalCurrent(){
		return super.getTotalCurrent();
	}
	double totalPower(){
		return super.getTotalPower();
	}
	double totalEnergy(){
		return super.getTotalEnergy();
	}
}
