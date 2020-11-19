/*****************************************************************************************************************
 * Description: This code stablish a connection with the raspberry pi and depending on the action that this one *
 * sets, then the system will set the corresponding light
 * ***************************************************************************************************************
 * Author: Ane San Martin Igarza                                                                                 *
 * ***************************************************************************************************************
 * Last Update: 23/06/2020                                                                                       *
 * **************************************************************************************************************/
// define the library needed
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// DEfine the pin used for the strip of leds
#define LED_PIN   12

// Number of leds on the strip
#define LED_COUNT 14

// Declaramos el objeto TIRA NEOPIXEL:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


String action = "none";
bool put_acolor = true;

void setup() {
    Serial.begin(115200);
// This line is specified in order that Adafruit Trinket supports 5V 16 MHz.
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  strip.begin();           // initialize the STRIP NEOPIXEL(REQUIRED)
  strip.show();            // TURN OFF the STRIP
  strip.setBrightness(50); // set the brightness (aproximatly al 1/5 (max = 255))
}


void loop() {
  //received the action that must be performed from the Raspberry board
    if (Serial.available() > 0) {
    action = Serial.readStringUntil('\n');
    Serial.print("received");
    put_acolor=true;
  }
  //depending on the color use the correspondent function to set the desired color
  put_acolor = false;
  if(action == "init1" || action == "excited_attract")
    colorWipe(strip.Color(random(0,255), random(0,255), random(0,255)), 10);
  if(action ==  "init2" || action == "interested_excited")
    colorWipe(strip.Color(0,   255, 0), 10); // green
  if(action ==  "init3" || action == "happy")
    rainbowane();
  if(action == "rotate")
    colorWipe(strip.Color(255,   0, 255), 10); // Magenta
  if(action ==  "move")
    colorWipe(strip.Color(0, 255, 0), 10); // green
  if(action ==  "scared")
    colorWipe(strip.Color(  0,   0, 255), 10);//Blue
  if(action == "very_scared")
    colorWipe(strip.Color(  0,   0, 255), 10);//Blue
  if (action == "sad")
    colorWipe(strip.Color(  0, 255, 255), 10); // Cyan
  if(action == "angry")
    colorWipe(strip.Color(  255,   0, 0), 10);//red
  if (action ==  "none" )
    turn_off();
}



uint32_t rainbow(){
return strip.Color(random(0,255), random(0,255), random(0,255));
}

//set each pixel to a different color
uint32_t rainbowane(){
for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i,strip.Color(random(0,255), random(0,255), random(0,255)));// it must be turned on one by one
    strip.show();// Actualize the strip
    delay(10);// set a small delay
  }
}
uint32_t turn_off(){
  for(int i=0; i<strip.numPixels(); i++) { 
    strip.setPixelColor(i,strip.Color(0, 0, 0));        
    strip.show();                          
    delay(10);                          
  }
  }
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // set a fixed color to each led of the strip
    strip.setPixelColor(i, color);         
    strip.show();                          // Actualize the strip
    delay(wait);                           //make a small delay
  }
}

void theaterChase(uint32_t color, int wait) {
  // Repite 10 veces...
  for(int a=0; a<1; a++) {  
    //  'b' cuenta de 0 a 2...
    for(int b=0; b<3; b++) { 
      //   Establece todos los píxeles en RAM a 0 (apagado)
      strip.clear();         
      // 'c' cuenta desde 'b' hasta el final de la tira en pasos de 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // Establece el pixel 'c' con el valor 'color'
        strip.setPixelColor(c, color); 
      }
      // Actualización de la tira con los nuevos contenidos
      strip.show(); 
      // Hace una pausa
      delay(wait);  
    }
  }
}

// Ciclo del arco iris a lo largo de toda la tira. 
// Pasa el tiempo de retraso (en ms) entre cuadros.
void rainbow(int wait) {
// El tono del primer píxel ejecuta 5 bucles completos a través de la rueda de colores. 
// La rueda de colores tiene un rango de 65536 pero está OK si le damos la vuelta, así que solo cuenta de 0 a 5 * 65536. 
// Agregar 256 a firstPixelHue cada vez, significa que haremos 5 * 65536/256 = 1280 pases a través de este bucle externo:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { 
      // Para cada píxel en la tira... 
      // Compense el tono del píxel en una cantidad para hacer una revolución completa 
      // de la rueda de colores (rango de 65536) a lo largo de la tira (strip.numPixels() pasos)  
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
