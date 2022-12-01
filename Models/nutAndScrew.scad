use <threads-library-by-cuiso-v1.scad>

headDiameter = 10;
headThickness = 3;
threadDiameter = 7;
threadLength = 30;

grooveThickness = 3;
grooveDepth = 1;


difference()
{
cylinder(h=headThickness, r=headDiameter/2, $fn = 32);
cube(headDiameter, grooveThickness, grooveDepth);
}


translate([0,0,headThickness])
thread_for_screw(threadDiameter, threadLength);