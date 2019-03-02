/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.BufferedReader;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.FileReader;
/**
 * Add your docs here.
 */
public class IOWithRIO {
    File f;
    BufferedWriter bw;
    FileWriter fw;
    BufferedReader br;

    public IOWithRIO(){
    try{
        f = new File("/home/lvuser/Output.txt");
        if(!f.exists()){
            f.createNewFile();
        }
        fw = new FileWriter(f);
    } catch(IOException e){
        e.printStackTrace();
    }
    bw = new BufferedWriter(fw);
    }

    public void writeToRIO(double armReading, double wristReading){
        try{
            bw.write("hi");
            bw.close();
            fw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }
    public void readFromRIO(){
        try{
            br = new BufferedReader(new FileReader(f));
            String currentLine = br.readLine();
            SmartDashboard.putString("read string:", currentLine);
            br.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }
}