$fn=64;
peiceHeight = 65;
peiceLength = 50;

holeSize = 4;
holeSpacing = 15;
holeOffset = 18;
numHoles = 3;

// points for naca fin taken from here:
/// http://chaaawa.com/airfoils/export.cgi?f=e837
e837_points = [[1000,0],[996.25,0.22],[985.3399,1.18],[967.95,3.16],[944.62,6.07],[915.69,9.87],[881.64,14.74],[843.19,20.71],[801.09,27.69],[756.15,35.48],[709.12,43.78],[660.78,52.22],[611.83,60.28],[562.71,67.37],[513.65,73.1],[464.91,77.26],[416.74,79.75],[369.43,80.53],[323.23,79.71],[278.54,77.49],[235.82,74],[195.54,69.35],[158.1,63.67],[123.89,57.05],[93.25,49.6399],[66.4899,41.59],[43.88,33.07],[25.64,24.3099],[11.98,15.56],[3.16,7.2],[0,0],[3.16,-7.2],[11.98,-15.56],[25.64,-24.31],[43.88,-33.07],[66.4899,-41.59],[93.25,-49.64],[123.89,-57.05],[158.1,-63.67],[195.54,-69.35],[235.82,-74],[278.54,-77.49],[323.23,-79.71],[369.43,-80.53],[416.74,-79.75],[464.91,-77.26],[513.65,-73.1],[562.71,-67.37],[611.83,-60.28],[660.78,-52.22],[709.12,-43.78],[756.15,-35.48],[801.09,-27.69],[843.19,-20.71],[881.64,-14.74],[915.69,-9.87],[944.62,-6.07],[967.95,-3.16],[985.3399,-1.18],[996.25,-0.22]];

difference() {   

    linear_extrude(height=peiceHeight)
    scale([peiceLength, peiceLength])
    scale (0.0018)
    polygon(points=e837_points);

    // Create cuts for support to go through.
    for(i = [0 : numHoles -1])
    {
        translate([holeOffset + holeSpacing * i, 0,0])
        cylinder(h= peiceHeight + 1 , r = holeSize);
    }
}



