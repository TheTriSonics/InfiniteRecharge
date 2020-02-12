/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.io.*;
import java.util.*;

/**
 * Add your docs here.
 */
public class CSVReader {
    String filename;
    double vmax;

    public CSVReader(String filename) {
	    this.filename = filename;
    }

    public double[][] parseCSV() {
	    File file= new File(filename);
	
	    List<List<String>> lines = new ArrayList<>();
	    Scanner inputStream;
	
	    try{
	        inputStream = new Scanner(file);
            vmax = Double.parseDouble(inputStream.nextLine().trim());
	        while(inputStream.hasNextLine()){
	    	    String line= inputStream.nextLine();
	    	    String[] values = line.split(",");
	    	    lines.add(Arrays.asList(values));
	        }
	        inputStream.close();
	    } catch (FileNotFoundException e) {
			e.printStackTrace();
			System.out.println("Error loading " + filename);
	    }
	    double [][] profile = new double[lines.size()][5];
	    for(int j = 0; j < lines.size(); j++) {
	        List<String> line = lines.get(j);
	        for (int i = 0; i < 5; i++) {
	    	    profile[j][i] = Double.parseDouble(line.get(i));
	        }
		}
	    return profile;
    }
    
    public double getVmax() {
	    return vmax;
    }

    public static void main(String[] args) {
	    String filename = args[0];
	    CSVReader reader = new CSVReader(filename);
        reader.parseCSV();
    }
}
