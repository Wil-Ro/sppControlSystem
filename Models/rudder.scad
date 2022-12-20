frontLength=10;
backLength=40;
height=120;
tipWidth=1.5;
cutHeight = 30;
cutWidth = 9;
cutLength = 20;
armLength = 10;
armWidth = 69;
armHeight = 6;
midWidth=9;  
axle=2.6;  
servoHole = 2.6;

$fn=200;
union(){
    difference(){ //arm for servo control
        difference(){
           translate([-4.5,-(armWidth/2),height-armHeight]) cube([armLength,armWidth,armHeight]);
           translate([0,-30,height-armHeight]) cylinder(h=armHeight+5,r=servoHole/2);
           translate([0,30,height-armHeight]) cylinder(h=armHeight+5,r=servoHole/2); 
        }
        translate([0,0,-1])
        cylinder(height+2,axle,axle); //main hole in centre
    }
    difference(){    //rudder with cut section
            difference()
            {
                hull()
                {
                    translate([frontLength,0,0])
                       cylinder(height,tipWidth/2,tipWidth/2);
                 
                    translate([-backLength,0,0]) 
                        cylinder(height,tipWidth/2,tipWidth/2);

                   cylinder(height,midWidth/2,midWidth/2);
                 }

                translate([0,0,-1])
                cylinder(height+2,axle,axle);
           }
           translate([-4.5,-10,height-35]) cube(cutLength,cutWidth,cutHeight); //cut out section
       
    }
}