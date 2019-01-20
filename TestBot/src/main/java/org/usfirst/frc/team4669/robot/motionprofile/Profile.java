package org.usfirst.frc.team4669.robot.motionprofile;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Scanner;

public class Profile {

	// Position (rotations) Velocity (RPM) Duration (ms)
	private double[][]pointsL;
	private double[][]pointsR;
	
	public Profile(double [][] pointsL,double[][] pointsR){
		this.pointsL = pointsL;
		this.pointsR = pointsR;
	}
	
	public Profile(){
		
	}
	
	public Profile(String CSVLocL,String CSVLocR){
		fillPoints(CSVLocL,CSVLocR);
	}
	
	public double[][] getLeft(){
		return pointsL;
	}
	public double[][] getRight(){
		return pointsR;
	}
	public int kNumPointsL(){
		return pointsL.length;
	}
	public int kNumPointsR(){
		return pointsR.length;
	}
	
	public void fillPoints(String fileLocL,String fileLocR) {
		pointsL = CSVtoArr(fileLocL);
		pointsR = CSVtoArr(fileLocR);
	}
	
	private double[][] CSVtoArr(String fileLoc) {
		Scanner scanIn = null;
		int row = 0,count=0;
		String inputLine= "";
		double[][] doubleArr = null;
		System.out.println("Setting up array from CSV");
		
		try {
			//Sets up a scanner
			scanIn = new Scanner(new BufferedReader(new FileReader(fileLoc)));
			
			while (scanIn.hasNextLine()) {
				//Increases the count of how many lines are in the CSV
				count++;
			}
			
			doubleArr = new double[3][count];
			
			while (scanIn.hasNextLine()) {
				//Reads line from file
				inputLine = scanIn.nextLine();
				//Splits line into string array elements at commas
				String[] InArray = inputLine.split(",");
				
				//Copies string array into the double array
				for(int x=0; x < InArray.length; x++) {
					doubleArr[row][x] = Double.parseDouble(InArray[x]);
				}
				//Increments the row in double array
				row++;
			}
		} catch (Exception e){
			System.out.println(e);
		}
		
		return doubleArr;
	}
	
}
