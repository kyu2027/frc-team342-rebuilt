// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class AprilTagIDs {

    //Holds all IDs for the hub
    public enum hubTagsIDs {
        FRONT_CENTER(0, 0),
        FRONT_OFFSET(0, 0),
        RIGHT_CENTER(0, 0),
        RIGHT_OFFSET(0, 0),
        LEFT_CENTER(0, 0),
        LEFT_OFFSET(0, 0),
        BACK_CENTER(0, 0),
        BACK_OFFSET(0, 0);

        private int blueHubTagID;
        private int redHubTagID;

        hubTagsIDs(int blueHubTagID, int redHubTagID) {
            this.blueHubTagID = blueHubTagID;
            this.redHubTagID = redHubTagID;
        }

        //Gets the ID for hub tag when on blue alliance
        public int getBlueHubTagID() {
            return blueHubTagID;
        }

        //Gets the ID for hub tag when on red alliance
        public int getRedHubTagID() {
            return redHubTagID;
        }
    }

    //Holds all IDs for the outpost
    public enum outpostTagsIDs {
        CENTER(0, 0),
        OFFSET(0, 0);

        private int blueOutpostTagID;
        private int redOutpostTagID;

        outpostTagsIDs(int blueOutpostTagID, int redOutpostTagID) {
            this.blueOutpostTagID = blueOutpostTagID;
            this.redOutpostTagID = redOutpostTagID;
        }

        //Gets the ID for outpost tag when on blue alliance
        public int getBlueOutpostTagID() {
            return blueOutpostTagID;
        }

        //Gets the ID for outpost tag when on red alliance
        public int getRedOutpostTagID() {
            return redOutpostTagID;
        }
    }

    //Holds all IDs for the tower
    public enum towerTagsIDs {
        CENTER(0, 0),
        OFFSET(0, 0);

        private int blueTowerTagID;
        private int redTowerTagID;

        towerTagsIDs(int blueTowerTagID, int redTowerTagID) {
            this.blueTowerTagID = blueTowerTagID;
            this.redTowerTagID = redTowerTagID;
        }

        //Gets the ID for tower tag when on blue alliance
        public int getBlueTowerTagID() {
            return blueTowerTagID;
        }

        //Gets the ID for tower tag when on red alliance
        public int getRedTowerTagID() {
            return redTowerTagID;
        }
    }

}
