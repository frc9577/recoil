package frc.robot.utils;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.commands.PathPlannerAuto;

// This goes to "/home/lvuser/deploy/pathplanner/autos" and grabs the names of all .auto files
// We are asuming that the PathPlannerAuto(string) is the file name of the auto
// We are asuming that the files are always in the "/home/lvuser/deploy/pathplanner/autos"
// We found this information at https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
public class AutoCommands {
    public static ArrayList<String> getAutoNames() {
        // Creating an ArrayList so I can fill it with auto file names w/o the extention.
        ArrayList<String> autoNames = new ArrayList<String>();

        // Getting the folder and files.
        String deployDirectoryPath = Filesystem.getDeployDirectory().getPath();
        File deployDirectory = new File(deployDirectoryPath+"/pathplanner/autos");
        File[] listOfFiles = deployDirectory.listFiles();

        // Checking if the files exist
        if (listOfFiles != null) {
            for (File file : listOfFiles) {
                String fullName = file.getName(); // Returns [file_name].[extention]
                if (file.isFile()) {
                    String[] splitName = fullName.split("\\."); // Splitting the file name into seprate parts.
                    
                    // If its an auto file then add it to the list.
                    if (splitName.length > 1 && "auto".equals(splitName[1])) {
                        autoNames.add(splitName[0]);
                        System.out.println("FOUND AUTO: "+splitName[0]);
                    }
                }
            }
        }

        // Returning the new array
        return autoNames;
    }

    public static ArrayList<Command> getAutoCommands(Optional<DriveSubsystem> driveSubsystem) {
        ArrayList<Command> autoCommands = new ArrayList<Command>();

        if (driveSubsystem.isPresent()) {
            ArrayList<String> autoNames = getAutoNames();
            for (String autoName : autoNames) {
                autoCommands.add(new PathPlannerAuto(autoName));
            }
        }

        return autoCommands;
    }
}
