width = 70;
depth = 70;
height = 32;

sWidth = 14;
sDepth = 32;
sHeight = 5;

cWidth = 10;



// top surface
difference()
{
    // main
    cube([width, depth, 5]);    
    
      union()
    {
        // edge cutout
        translate([0, 10, 0])
        cube([cWidth, 50, 5]);
        
         // edge cutout
        translate([width-cWidth, 10, 0])
        cube([cWidth, 50, 5]);
        
        // servo hole
        translate([(width/2)-(sWidth/2), 30, 0])
        cube([sWidth, sDepth, sHeight]);
    }
}


// side surface
difference()
{
    // main
    translate([0, 0, -height])
    cube([width, 5, height]);
    
    union()
    {
        // edge cutout
        translate([0, 0, -(height+5)])
        cube([cWidth, 10, 27]);
        
         // edge cutout
        translate([width-cWidth, 0, -(height+5)])
        cube([cWidth, 10, 27]);
    }
    
}
