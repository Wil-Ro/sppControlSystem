$fn=200;



translate([0,-10,0])
union(){
    difference(){
        cube([shape_width, shape_depth, shape_height]);
        union(){
        
        translate([0,-15,0])
        cube([shape_width,10,10]);
        
        translate([0,-20,0])
rotate([295,0,0])
        cube([shape_width, shape_depth / 8, sqrt(shape_width^2 + shape_height^2)]);
        }
        translate([0,shape_depth,0])
rotate([65,0,0])
        cube([shape_width, shape_depth / 8, sqrt(shape_width^2 + shape_height^2)]);
        
        
        rotate([0,0,90])
        translate([7,-shape_width / 2,0])
        for(i = [0 : numHoles -1]){
            translate([holeOffset + holeSpacing * i, 0,0])
            cylinder(h= shape_height + 1 , r = holeSize);
        }
    }
    
    

}

shape_depth = 100;
shape_width = 60;;
shape_height = 10;

circle_radius = 25;

holeSize = 4;
holeSpacing = 15;
holeOffset = 32;
numHoles = 3;

/*union(){
    difference(){
        translate([sphere_radius, sphere_radius])
        sphere(sphere_radius);
        
        translate([0, 0, sphere_radius * -2])
        cube(sphere_radius * 2);
    }
    
}
//translate([0,0,sphere_radius *-2])
//cube(sphere_radius * 2);

sphere_radius = 10;
*/