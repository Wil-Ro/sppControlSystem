$fn = 30; // number of faces in screw holes

thickness = 5; // thickness, this doesnt work

// main body
width = 70;
depth = 70;
height = 32;

// servo
sWidth = 15;
sDepth = 32;
sHeight = 5;

cWidth = 10; // cutout width

sDiameter = 5; // screw diameter

rDiameter  = 10; //rudder rod diameter (readjust)

// top surface
difference()
{
    // main
    cube([width, depth, thickness]);    
    
      union()
    {
        // edge cutout
        translate([0, 10+thickness, 0])
        cube([cWidth, 45, thickness]);
        
         // edge cutout
        translate([width-cWidth, 10+thickness, 0])
        cube([cWidth, 45, thickness]);
        
        // servo hole
        translate([(width/2)-(sWidth/2), 30, 0])
        cube([sWidth, sDepth, sHeight]);
        
        // screw holes
        
        // upper left
        translate([cWidth/2, width-cWidth/2, 0])
        cylinder(h=thickness, d=sDiameter);
        // upper right
        translate([depth-cWidth/2, width-cWidth/2, 0])
        cylinder(h=thickness, d=sDiameter);
        
       
        // lower right
        translate([depth-cWidth/2, cWidth-2.5, 0])
        cylinder(h=thickness, d=sDiameter);
        // lower left
        translate([cWidth/2, cWidth-2.5, 0])
        cylinder(h=thickness, d=sDiameter);
        
        
        //rudder hole
        translate([(width/2),15,0])
        cylinder(h=thickness,d=rDiameter);
    }
}


