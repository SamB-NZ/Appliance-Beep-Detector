#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;

// =====================================================
// Arduino Goertzel Beep Detector
// =====================================================

//Built to detect end of cycle appliance beeps and send a notification.
//Written to work on an Arduino Uno R4 WiFi. Sketch below does not include Arduino cloud code for notification.
//Core functionality is ADC sample collection > Goertzel analysis for target frequencies > State machine using thresholds and some smoothing to detect target beep pattern.
//My particular case has 3x 1s beeps with 1s pauses. The beeps have two peak tones at 2400Hz & 4800Hz and the sketch uses both tones in the state machine. Can be adjusted to mono tone if required.
//Graphic EQ using R4 Wifi LED matrix included for interest / fun.
//Additional hardware required is an electret microphone, I used an Adafruit MAX9814.
//Thresholds were tuned using a mirror Python script to replay repeatable WAV files, including some with added background noise. Workflow for that was to record WAV in Python from Arduino via serial.

// =====================================================
// =================   DEBUG TOGGLES  ==================
// =====================================================
const bool DEBUG_TIMING = false;    //Set to true for monitoring processing time within loop
const bool DEBUG_DETECTOR = true;  //Set to true for logging key detector metrics in serial. Pairs with Python script "beep_detector_logger.py" for testing and visualisation.

// =====================================================
// ===============   SAMPLE COLLECTION  ================
// =====================================================
const int micPin = A0;                              //Set up micPin on A0
const float sampleRate = 20000.0;                   //Sample rate for Goertzel
const unsigned long period = 1000000UL/sampleRate;  //time between samples being taken for the Goertzel analysis
unsigned long lastSampleTime = 0;                   //Used to control sampling frequency
const int N = 1024;                                 //Sample count for Goertzel
float samples[N];                                   //Array used to store the samples for later Goertzel analysis

// =====================================================
// ===========   BEEP DETECTING PARAMETERS  ============
// =====================================================
const float fL = 2400.0;  //Target frequency of lower tone.
float omegaL;             //Used later in pre-computed Goertzel parameters
float coeffL;             //Used later in pre-computed Goertzel parameters
const float fH = 4800.0;  //Target frequency of higher tone
float omegaH;             //Used later in pre-computed Goertzel parameters
float coeffH;             //Used later in pre-computed Goertzel parameters
const float fR = 1000.0;  //Reference background noise tone
float omegaR;             //Used later in pre-computed Goertzel parameters
float coeffR;             //Used later in pre-computed Goertzel parameters
int thresholdLow = 2000;  //Threshold for 2400Hz beep trigger above rolling average baseline
int thresholdHigh = 2000; //Threshold for 4800Hz beep trigger above rolling average baseline
int tolL = 0;             //Used for detecting end of possible 2400Hz beep
int tolH = 0;             //Used for detecting end of possible 4800Hz beep
unsigned long beepSeqTimer = 0; //Watchdog timer to clear stale partial beep sequence

// =====================================================
// ========   SETUP FOR BEEP DETECTOR MACHINE  =========
// =====================================================

bool endCycle = false;              //Gets triggered true when the end of cycle beeps have been detected (beepSeq = [1,1,1,1,1])
int beepSeq[5] = {0};               //Tracks detected beep/pause pattern: [beep, pause, beep, pause, beep]
bool possbeep = 0;                  //Used to trigger if a beep might be in process
unsigned long possbeeptimer = 0;    //For timing the length of a possible beep
bool posspause = 0;                 //Used to trigger if a pause might be in process
unsigned long posspausetimer = 0;   //For timing the length of a possible pause
unsigned long endCycleTimer = 0;    //Used to hold things up at the end of the cycle to ignore reminder beeps & prevent repeat notifications
const int AAL = 3;                  //Averaging Array Length - used for small rolling average of 'live' frequency magnitudes to damp out noise factors
float magHistoryL[AAL] = {0};       //Establish an array of length AAL for storing the low frequency magnitude history. Set it to 0.
float magHistoryH[AAL] = {0};       //Establish an array of length AAL for storing the high frequency magnitude history. Set it to 0.
float magHistoryR[AAL] = {0};       //Establish an array of length AAL for storing the reference frequency magnitude history. Set it to 0.
int k = 0;                          //Establish an indexing term k that rolls around to fill the magHistory arrays.
int count = 1;                      //Used on initial run to keep track of the magnitude array filling and to take a correct average. Stops indexing up when it = AAL
float emaUP = 0.08;                 //Asymmetric EMA parameter for the running average  / baseline frequency magnitude.    
float emaDWN = 0.3;                 //Asymmetric EMA parameter for the running average / baseline frequency magnitude. 
float magLAvg = 0;                  //Set up the running average variable for the lower frequency magnitude
float magHAvg = 0;                  //Set up the running average variable for the higher frequency magnitude
float magRAvg = 0;                  //Set up the running average variable for the reference frequency magnitude
float magl = 0;                     // Set up the variable for the live lower frequency magnitude
float magh = 0;                     // Set up the variable for the live higher frequency magnitude
float magr = 0;                     // Set up the variable for the live reference frequency magnitude

// =====================================================
// ==============   CREATE EMA FUNCTION  ===============
// =====================================================
//Asymmetric EMA used to rise slowly and drop quickly.
//Slower rise avoids accumulation across beeps but needs to be reactive enough to detect end of beep edge.
float ema_asym(float new_s, float avg_s, float up, float down) {
  float a;
  if (new_s > avg_s) a = up;
  else a = down;
  return (a * new_s) + ((1.0f - a) * avg_s);
}

// =====================================================
// ====   SETUP FOR RECENTERING DC OFFSET IN SETUP  ====
// =====================================================
long sum = 0;
float reCenter = 0;

// =====================================================
// ========   SETUP REUSABLE GOERTZEL FUNCTION  ========
// =====================================================
//Note this function required coeff to be pre-computed
float goertzelMag(int N, float coeff, const float samples[]) {
  float s = 0;
  float s_prev = 0;
  float s_prev2 = 0;
  for (int i = 0; i < N; i++) {
    s = samples[i] + (s_prev * coeff) - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }
  return sqrt((s_prev * s_prev) + (s_prev2 * s_prev2) - (coeff * s_prev * s_prev2));
}

// =====================================================
// ===========   SETUP EQ DISPLAY VARIABLES  ===========
// =====================================================
uint8_t frame[8][12] = {0};       //Matches size & shape of Uno R4 Wifi LED matrix (12 columns, each 8 LEDs tall). 1 is LED on, 0 is LED off.
float magBins[12] = {0};          //12 frequency bins, one per LED matrix column
float emaUPEQ = 0.1;              //Asymmetric slow up / fast down EMA to avoid too much accumulation and keep it anchored to a baseline
float emaDWNEQ = 0.25;
unsigned long EQTimer = 0;        //Used for controlling refresh rate of EQ display
const unsigned long EQRate = 100; //Sets refresh rate of EQ display    
float EQCoeffs[12];               //Array for storing the Goertzel coefficients for each frequency bin displayed
const float EQfreqs[12] = {       //12 frequencies to display. Display is for visual interest, choose any frequencies you like.
  250,   // 0
  350,   // 1
  500,   // 2
  700,   // 3
  1000,  // 4 - fR
  1400,  // 5
  1700,  // 6
  2000,  // 7 - fL
  2400,  // 8
  3000,  // 9
  3800,  //10 - fH
  4800   //11
};

// =====================================================
// ===========   SETUP EQ DISPLAY FUNCTION  ============
// =====================================================
void drawEQ(float bins[12]) {         //Compute the 12 frequency magnitudes in a previous step then pass them into this EQ display rendering function as 'bins[]'
  float logBins[12];                  //An array of log magnitudes to display instead of raw mags. Closer to dB scale output.

  // Convert to log scale (dB-ish)
  for (int i = 0; i < 12; i++) {
    float m = bins[i];
    if (m < 1.0f) m = 1.0f;          // avoid log(0)
    float soft = powf(m, 0.7f);      // compress the magnitudes by raising to the power of 0.7. Reduces dynamic range and stops strong signals drowning out the rest.
    logBins[i] = 20.0f * log10f(soft);    // may need <math.h>
  }

  //---Find min & max. Used later for normalising display.---
  float maxMag = logBins[0];
  float minMag = logBins[0];
  for (int i = 1; i < 12; i++) {
    if (logBins[i] > maxMag) maxMag = logBins[i];
    if (logBins[i] < minMag) minMag = logBins[i];
  }
  //---Calculate range and protect against 0 division---
  float range = (maxMag - minMag);
  if (range < 0.0001f) range = 0.0001f;
  //---Reset all values in frame to 0---
  memset(frame, 0, sizeof(frame));
  //---Set frame columns to appropriate heights, lowest value being bottom LED off---
  //---K indexes columns, i.e. frequency bins. Nested loops populate frame with LED heights one frequency bin at a time.
  for (int k = 0; k < 12; k++) {
    float norm = (logBins[k] - minMag) / range; //normalise value at each column 0-1
    int height = int(norm * 8 + 0.0001f); // Convert normalized value to LED height (0–8). Small offset avoids float rounding errors causing values like 2.999999 to truncate to 2.

     // clamp to valid range just in case
    if (height < 0) height = 0;
    if (height > 8) height = 8;
    
    //Populate the display frame with 1s to the appropriate height in each frequency column.
    //The display frame addresses count top down, i.e. frame[row][col], row 0 = top LED, row 7 = bottom LED
    //Therefore to find the index address need to subtract the height from 8
    // H / I
    // 8 / 0 -- Top LED 
    // 7 / 1
    // 6 / 2
    // 5 / 3
    // 4 / 4
    // 3 / 5
    // 2 / 6
    // 1 / 7 -- Bottom LED
   
    int magHeightIndex = 8 - height; // Subtracting height from 8 to get the index as explained above.
    for (int i = 7; i >= magHeightIndex; i--) {  //starting from bottom (index 7) turn LEDs on until index height is reached. When index = 8 (i.e. no LED) loop exits immediately.
      frame[i][k] = 1;
    }
  }
  matrix.renderBitmap(frame, 8, 12); //Arduino function to render the LED matrix based on values stored in frame
}

// =====================================================
// ===============   SETUP BLINK TIMER  ================
// =====================================================
//---Used to blink the LED when in the hold state, otherwise no on device feedback---
unsigned long blinkTimer = 0;
unsigned long blinkTime = 1000;
bool ledState = false;

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ====================   SETUP  ==================================================================================================================================================================
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
  Serial.begin(115200);
  delay(1500);  // Give USB and host time to sync
  matrix.begin();
  //Serial.println("possbeep,posspause,magHAvg,magLAvg,magh,magl,magprevl,magprevh");

  //---Precompute main Coeffs---
  omegaL = 2.0 * PI * fL / sampleRate;
  coeffL = 2.0 * cos(omegaL);
  omegaH = 2.0 * PI * fH / sampleRate;
  coeffH = 2.0 * cos(omegaH);
  omegaR = 2.0 * PI * fR / sampleRate;
  coeffR = 2.0 * cos(omegaR);

  //---Compute Coeffs for EQ Display---
  for (int i = 0; i < 12; i++) {
    float omega = 2.0 * PI * EQfreqs[i] / sampleRate;
    EQCoeffs[i] = 2.0 * cos(omega);
  }

  ///---Compute recenter value---
  sum = 0;
  // take a centering average of the mic reading
  for(int j = 0; j < N; j++) {
    sum += analogRead(micPin);
  }
  reCenter = sum/N;

  pinMode(LED_BUILTIN, OUTPUT); // set up LED_BUILTIN as an output, use it to blink when endCycle = 1
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// ===============================================================   MAIN LOOP  ===================================================================================================================
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void loop() {

  unsigned long t1 = micros();
  // =====================================================
  // ===============   SAMPLE COLLECTION  ================
  // =====================================================
  //gather a sample set for subsequent Goertzel analysis
  //sampleTakeTime = micros();
  lastSampleTime = micros();
  for(int i = 0; i < N; i++) {
    while (micros() - lastSampleTime < period);
    lastSampleTime += period;
    int raw = analogRead(micPin);
    samples[i] = raw - reCenter;
  }

  unsigned long t2 = micros();

  // =====================================================
  // ===============   END OF CYCLE HOLD  ================
  // =====================================================
  //Hold the process for 20 min in a non-blocking way if endCycle = true to avoid triggering again from the reminder beeps
  //Runs the EQ display during the hold
  if (endCycle == true && (millis() - endCycleTimer) < (1000UL * 60UL * 20UL)) {
      if (millis() - EQTimer >= EQRate) {                                           //Controls EQ refresh rate
        for (int i = 0; i < 12; i++) {                                              //Loop runs the Goertzel calc on each EQ freq and updates the smoothed EMA  
          float gEQ = goertzelMag(N, EQCoeffs[i], samples);
          magBins[i] = ema_asym(gEQ, magBins[i], emaUPEQ, emaDWNEQ);
        }                                       
    drawEQ(magBins);    
    EQTimer = millis();
    }

    // flash built in LED when in hold state
    if (millis() - blinkTimer > blinkTime) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
      blinkTimer = millis();
    }
  
      return; //return goes back to the start of the void loop.
  }
    else if (endCycle == true && (millis() - endCycleTimer) > (1000UL * 60UL * 20UL)) {
    endCycle = false;
  }
  // =====================================================
  // =======  GOERTZEL ANALYSIS & HISTORY STORAGE  =======
  // =====================================================

  // =======  LOW FREQUENCY  =======
 
  //calculate L magnitude
  magl = goertzelMag(N, coeffL, samples);
  magLAvg = ema_asym(magl, magLAvg, emaUP, emaDWN);
  magHistoryL[k] = magl;

  // =======  HIGH FREQUENCY  =======  
  magh =  goertzelMag(N, coeffH, samples);
  magHAvg = ema_asym(magh, magHAvg, emaUP, emaDWN);
  magHistoryH[k] = magh;

  // =======  REFERENCE FREQUENCY  =======
  magr = goertzelMag(N, coeffR, samples);
  magRAvg = ema_asym(magr, magRAvg, emaUP, emaDWN);
  magHistoryR[k] = magr;
  
  k = (k + 1) % AAL; // circular buffer for k
  
  unsigned long t3 = micros();

  // =====================================================
  // =============  BEEP DETECTING MACHINE  ==============
  // =====================================================
  
  //Next phase is about detecting the beep
  //Plan is to compare latest H & L magnitude, 'magh' & 'magl' to the averages 'magHAvg' & magLAvg'. If above threshold consider that a beep may have started
  //If a beep may have started, monitor that H & L magnitudes stay high within a tolerance for 750 - 3200ms.
  //If they do meet that condition set beep detect 'beepSeq' array to 1 in index location
  
  //Create previous mag history index pointers
  int prevkH = (k - 2 + AAL) % AAL;
  int prevkL = (k - 2 + AAL) % AAL;
  int prev2kH = (k - 3 + AAL) % AAL;
  int prev2kL = (k - 3 + AAL) % AAL;
  //Smooth out magh & magl by averaging over a few more past cycles
  magl = (magl + magHistoryL[prevkL] + magHistoryL[prev2kL])/count;
  magh = (magh + magHistoryH[prevkH] + magHistoryH[prev2kH])/count;
  magr = (magr + magHistoryR[prevkH] + magHistoryR[prev2kH])/count;
  //Update count to get the average correct while the array is filling. Stop increasing it when it is equal to AAL.
  if (count < AAL) count++;
  //Trigger possbeep = 1 if low and high magnitudes exceed threshold relative to baseline. Start a timer to monitor the beep length.
  if (possbeep == 0) {
    if ((magl - magLAvg) > thresholdLow && (magh - magHAvg) > thresholdHigh) {
    possbeep = 1;    
    possbeeptimer = millis();
    if (beepSeq[0] == 0) beepSeqTimer = millis();
    }
  }
  else if ((magl - magLAvg) < tolL || (magh - magHAvg) < tolH) {
      // possbeep == 1, therefore in a potential beep state. If above condition has been met, the possible beep can be considered over. Now test to see if it was a legit beep.
      possbeep = 0;
    if ((millis() - possbeeptimer) > 750UL && beepSeq[4] == 0) {
      //The above else-if has flagged the beep has ended. If it meets the timing criteria, a legit beep has been detected & it isn't an erroneous 4th beep. Set posspause state & end possbeeptimer. 
      posspause = 1;
      posspausetimer = millis();
      possbeeptimer = 0;        
      //find state of beepSeq array [0 (beep),1 (pause),2 (beep),3 (pause),4 (beep)]. Put 1 in the first free even number location in beepSeq.
      for (int i = 0; i < 5; i++) {
        if ((beepSeq[i] == 0) && i % 2 == 0) {
          beepSeq[i] = 1;
          break;
        } 
      } //close for / if for populating beepSeq 
    } //close legit beep detecting section. If a beep ended too soon, or was a '4th' beep, just ignore it, it doesn't count and the pause detector will deal with it. 
  }
  else if ((millis() - possbeeptimer) > 1300UL) {
        //possbeep == 1 and it hasn't ended. Check if it has exceeded allowable length - if it has, reject everything. Otherwise carry on.
        possbeep = 0;
        possbeeptimer = 0;
        posspause = 0;
        posspausetimer = 0;
        memset(beepSeq, 0, sizeof(beepSeq));
  }

  if (posspause == 1 && possbeep == 1 && ((millis() - posspausetimer) > 750UL) && (millis() - possbeeptimer) > 200UL && beepSeq[4] == 0) {
    //legit pause detected, i.e. it was in a possible pause state and a new beep has shown up at the right time for long enough to be probably legit. It also isn't an erroneous 4th beep.
    //put a 1 in the appropriate place in beepSeq (i.e. first free odd number location)
    posspause = 0;
    posspausetimer = 0;
    for (int i = 0; i < 5; i++) {
      if ((beepSeq[i] == 0) && (i % 2 == 1)) {
        beepSeq[i] = 1;
        break;
      }      
    }
  }
  else if (posspause == 1 && possbeep == 0 && beepSeq[4] == 0 && ((millis() - posspausetimer) > 1300UL)) {
    //No beep yet, and the final beep hasn't beeped, but the allowable pause time has passed. Reject everything.
    posspause = 0;
    posspausetimer = 0;
    memset(beepSeq, 0, sizeof(beepSeq));    
  }
  else if (posspause == 1 && possbeep == 0 && beepSeq[4] == 1 && ((millis() - posspausetimer) > 1300UL)) {
    //end of legit beep sequence condition met. Trigger end of cycle flag. Clear beep detector.
    endCycle = true;
    endCycleTimer = millis();
    posspause = 0;
    posspausetimer = 0;
    memset(beepSeq, 0, sizeof(beepSeq));
  }
  if (beepSeq[0] == 1 && (millis() - beepSeqTimer) > 7000) {
    //Watchdog timer if some condition results in a partially filled but stalled beepSeq array. Reset the array.
    memset(beepSeq, 0, sizeof(beepSeq));
  }

  unsigned long t4 = micros();

  // =====================================================
  // ===================  EQ DISPLAY  ====================
  // =====================================================
  
  if (millis() - EQTimer >= EQRate) {
    for (int i = 0; i < 12; i++) {
      float gEQ = goertzelMag(N, EQCoeffs[i], samples);
      magBins[i] = ema_asym(gEQ, magBins[i], emaUPEQ, emaDWNEQ);
    }
    drawEQ(magBins);    
    EQTimer = millis();
  }

  unsigned long t5 = micros();

  if (DEBUG_TIMING) {
    Serial.print("Samples = "); Serial.print(t2 - t1); Serial.print(",");
    Serial.print("Goertzel = "); Serial.print(t3 - t2); Serial.print(",");
    Serial.print("Detect = "); Serial.print(t4 - t3); Serial.print(",");
    Serial.print("EQ = "); Serial.print(t5 - t4); Serial.print(",");
    Serial.print("Total = "); Serial.println(t5 - t1);
  }

  if (DEBUG_DETECTOR) {
    Serial.print(possbeep); Serial.print(",");
    Serial.print(posspause); Serial.print(",");
    Serial.print(endCycle); Serial.print(",");
    Serial.print(magh - magHAvg); Serial.print(",");
    Serial.print(magl - magLAvg); Serial.print(",");
    Serial.print(magr - magRAvg); Serial.print(",");
    Serial.println(t2);
  }

}

 





