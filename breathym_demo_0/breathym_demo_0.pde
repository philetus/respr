/* REQUIRE THE NEWEST VERSION OF PROCESSING */

import processing.serial.*;

Serial myPort;        // The serial port
float xPos = 1;         // horizontal position of the graph
int readingMax = 355;  //adjust to the inByte reading boundary 
int readingMin = 320;
float xStep = 1; 

float[] brightlist;
int brightsmooth = 10; // set size of smoothing array
float brightness = 255; 
  
void setup () {
  // set the window size:
  size(900, 300);        
  
  brightlist = new float[brightsmooth]; 

  // List all the available serial ports
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 115200);
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');
  // set inital background:
  background(100);
}
void draw () {
  // everything happens in the serialEvent()
  float brightsum = 0.0;
  for (int i = 0; i < brightsmooth; i++) {
    brightsum += brightlist[i];
  }
  brightsum = brightsum / brightsmooth;
  background(brightsum);
  
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // convert to an int and map to the screen height:
    float inByte = float(inString); 
    print (inByte);
    
    // change color with inByte reading
      brightness =  mapToColor(inByte);
       println ("\t" + brightness);
     fill (brightness);
     
     // rotate brightness buffer
     for (int i = brightsmooth - 1; i > 0; i--) {
       brightlist[i] = brightlist[i - 1];
     }
     brightlist[0] = brightness;
     
     noStroke();
     rect (0,0,width, height*3/4);
     
    // draw the dot:
    stroke(200);
    point (xPos, mapToGraph(inByte) );
    
    // at the edge of the screen, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      background(0);
    } 
    else {
      // increment the horizontal position:
      xPos = xPos + xStep ;
    }
  }
}
float mapToColor(float y){
  float yTemp = map (y, readingMin, readingMax, 10, 245);
 return yTemp; 
}

//map inbyte to a point on graph
float mapToGraph (float y){
  float yTemp = map(y, readingMin, readingMax, 10, height/4-10);
  return height-yTemp;  
}