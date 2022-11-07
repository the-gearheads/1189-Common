package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

public class CameraParams {
    public int lastLatencyVal;
    public int equivalencyCount;
    public String name;
    public PhotonCamera camera;
    public boolean isConnected;

    public CameraParams(String name){
        camera = new PhotonCamera(name);
        this.name=name;
        this.equivalencyCount=0;
        this.lastLatencyVal=0;
        this.isConnected=false;
    }

    public void connect(){
        camera = new PhotonCamera(name);
        if(lastLatencyVal == camera.getLatestResult().getLatencyMillis())
          equivalencyCount++;
        else
          equivalencyCount=0;
        if(equivalencyCount>10)
          isConnected = false;
        else 
          isConnected = true;
    }
}
