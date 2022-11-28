$fn = 30; // number of faces in screw holes

thickness = 5; // thickness

// main body
width = 70;
depth = 70;
height = 32;

// servo
sWidth = 14;
sDepth = 32;
sHeight = 5;

cWidth = 10; // cutout width

sDiameter = 5; // screw diameter

// top surface
difference()
{
    // main
    cube([width, depth, 5]);    
    
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
        
        // NOTE: it might be worth getting rid of the 
        // top lower holes, we dont need both them and 
        // the side faces holes
        
        // lower left
        translate([cWidth/2, cWidth/2+thickness, 0])
        cylinder(h=thickness, d=sDiameter);
        // upper left
        translate([cWidth/2, width-cWidth/2, 0])
        cylinder(h=thickness, d=sDiameter);
        // lower right
        translate([depth-cWidth/2, cWidth/2+thickness, 0])
        cylinder(h=thickness, d=sDiameter);
        // upper right
        translate([depth-cWidth/2, width-cWidth/2, 0])
        cylinder(h=thickness, d=sDiameter);
    }
}


// side surface
difference()
{
    // main
    translate([0, 0, -height])
    cube([width, thickness, height]);
    
    union()
    {
        // edge cutout
        translate([0, 0, -(height+5)])
        cube([cWidth, 10, 27]);
        
         // edge cutout
        translate([width-cWidth, 0, -(height+5)])
        cube([cWidth, 10, 27]);
        
        // lower right
        translate([depth-cWidth/2, cWidth/2, -5])
        rotate([90, 0, 0])
        cylinder(h=thickness, d=sDiameter);
        
        // lower left
        translate([cWidth/2, cWidth/2, -5])
        rotate([90, 0, 0])
        cylinder(h=thickness, d=sDiameter);
        
    }
    
}
