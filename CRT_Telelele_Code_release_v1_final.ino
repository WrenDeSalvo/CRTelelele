/*
  Telelele
  A four-string touch instrument with a glide-aware looper.
  By Wren DeSalvo.

  ------------------------------------------------------------------
  HARDWARE
  ------------------------------------------------------------------
  Board:    Arduino Mega 2560
  Strings:  4x linear potentiometer, touch-sensitive (A0-A3)
  Audio:    toneAC library (not the standard tone()), pin 7, through
            a 47-ohm + 0.22uF low-pass filter into a guitar amp.
            toneAC is monophonic by hardware design -- only one pitch
            can sound at a time, in every mode.
  Encoder:  1x rotary encoder, full quadrature decoding
            CLK = pin 2, DT = pin 3 (both interrupt-capable on the Mega)
            button = pin 4 (INPUT_PULLUP)
  Display:  SSD1306 OLED, 0.96", 128x64, I2C address 0x3C
            Top 16px = status strip, bottom 48px = content area
            I2C runs at 400kHz only during boot, 25kHz otherwise

  ------------------------------------------------------------------
  ARCHITECTURE
  ------------------------------------------------------------------
  Single .ino file, no enums (Arduino's auto-prototype generator can
  break on function prototypes that reference a custom enum type
  declared later in the file -- every mode/state value here is a
  plain const int instead).

  loop() runs four sections every iteration, in this fixed order:
    1. Button handling     (clicks, holds, gesture detection)
    2. Encoder handling    (turns, smoothed and mode-dispatched)
    3. Mode audio behavior (reads the strings, drives toneAC)
    4. Display             (throttled to 30fps, see DISPLAY_FRAME_INTERVAL_MS)

  Modes (plain int, never enum):
    0 = Tuning            5 = Sound Tuning (one-shot triggers)
    1 = Record            6 = Tremolo
    2 = Playback Speed    7 = Vibrato
    3 = Playback Pitch    8 = Bitcrusher
    4 = Playback Fade

  Recording engine: one touched string is recorded at a time. A
  held, steady note costs a single stored event; only genuine pitch
  movement ("glide") is sampled continuously. See the comment block
  above MAX_EXACT_EVENTS for the full design rationale.

  Effects (Tremolo/Vibrato/Bitcrusher) run continuously in the
  background on whatever is currently playing, regardless of which
  mode screen is on display -- each is a simple on/off flag plus a
  single 0-100 knob that maps to two real underlying parameters.
*/

#include "pitches.h"
#include <toneAC.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int YELLOW_TOP = 0;
const int BLUE_TOP = 16;

// --- Pins ---
#define CLK 2
#define DT 3
#define BTN 4

// --- Thresholds & Timing ---
int threshold = 29;
const int idleTimeout = 300;
const int debounceDelay = 50;
const int clickWindow = 400;
const unsigned long saveSoundHoldThreshold = 4000;
const unsigned long eepromPromptHoldThreshold = 3000;
const unsigned long eepromConfirmHoldThreshold = 3000;
const unsigned long eepromAbortTimeout = 8000;
const unsigned long releaseFadeOutMs = 100;

// --- Post-exit cooldown ---
const unsigned long postExitCooldownMs = 600;
unsigned long exitedAtTime = 0;

// --- Load-from-EEPROM gesture (from Tuning mode only) ---
const unsigned long LOAD_CLICK_MAX_MS = 2000;
const unsigned long LOAD_HOLD_THRESHOLD_MS = 3000;
bool loadHoldFired = false;

bool inputLockedAfterLoad = false;

// --- Universal Exit Gesture: "frustration spam-click" detection ---
const unsigned long SPAM_CLICK_MAX_GAP_MS = 300;
int SPAM_CLICK_EXIT_COUNT = 6;
int spamClickCount = 0;
unsigned long lastSpamClickTime = 0;

// --- Tuning ---
int tuningPreset = 0;
bool knobActive = false;
unsigned long lastTurnTime = 0;

// --- Encoder (interrupt-driven, full quadrature state-table decoding) ---
// Both CLK and DT are watched on CHANGE (not just CLK on RISING), and
// every transition is validated against a fixed table of the 4 legal
// forward steps and 4 legal backward steps a clean encoder can produce.
// Any other old-state -> new-state combination is physically impossible
// for a working encoder and is silently discarded as noise, rather than
// being trusted the way a single instantaneous DT sample was before.
//
// A full mechanical detent cycles through all 4 valid states and returns
// to the rest state (encoded as 0 here). Only the final step BACK INTO
// the rest state counts as one completed tick -- the 3 intermediate
// steps are tracked (to keep the state machine correctly synced and to
// still reject genuinely invalid jumps) but don't themselves move
// encoderDelta. This keeps "one detent = one step," matching the feel
// of the original single-edge scheme, while keeping the noise rejection
// of full quadrature decoding.
volatile int encoderDelta = 0;
volatile uint8_t encoderLastState = 0;
volatile int8_t encoderCycleAccum = 0;

// Table indexed by (oldState << 2 | newState), oldState/newState each
// being (DT bit | CLK bit) -- DT is the high bit here, CLK the low bit,
// which sets the physical turn direction. Swap which pin is shifted left
// to reverse clockwise/counterclockwise if needed.
// +1 = valid forward sub-step, -1 = valid backward sub-step,
// 0 = invalid/impossible transition for a clean encoder (noise).
const int8_t ENCODER_TRANSITION_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// Set to -1 to reverse which way maps to increasing values,
// without touching the transition table or pin bit-ordering.
const int ENCODER_DIRECTION_SIGN = -1;

void handleEncoderChange() {
  uint8_t clkState = digitalRead(CLK);
  uint8_t dtState = digitalRead(DT);
  uint8_t newState = (dtState << 1) | clkState;
  uint8_t tableIndex = (encoderLastState << 2) | newState;
  int8_t step = ENCODER_TRANSITION_TABLE[tableIndex & 0x0F];
  encoderCycleAccum += step;
  if (newState == 0 && step != 0) {
    if (encoderCycleAccum >= 3) {
      encoderDelta += ENCODER_DIRECTION_SIGN;
    } else if (encoderCycleAccum <= -3) {
      encoderDelta -= ENCODER_DIRECTION_SIGN;
    }
    encoderCycleAccum = 0;
  }
  encoderLastState = newState;
}

// --- Encoder smoothing (rejects isolated opposite-direction glitches) ---
int recentTickDirections[3] = {0, 0, 0};
int pendingTickDirection = 0;
bool hasPendingTick = false;

// --- Click-priority suppression: a button press silences encoder ticks
// for a short window afterward, since pressing can mechanically nudge
// the encoder shaft on this hardware. ---
const unsigned long CLICK_TURN_SUPPRESS_MS = 300;
unsigned long lastButtonPressTime = 0;

// --- Tuning preset display names ---
const char* tuningNames[6] = {
  "BASS", "HORROR", "SOPRANO", "TENOR", "VIOLIN", "MAJ3RD"
};

// --- Modes ---
// 0 = Tuning
// 1 = Record
// 2 = Playback Speed
// 3 = Playback Pitch
// 4 = Playback Fade
// 5 = Sound Tuning
// 6 = Tremolo
// 7 = Vibrato
// 8 = Bitcrusher
int mode = 0;

// --- Button ---
int lastBtnState = HIGH;
unsigned long lastBtnTime = 0;
unsigned long btnHoldStart = 0;
bool btnHoldFired = false;
bool eepromPromptFired = false;
int clickCount = 0;
unsigned long firstClickTime = 0;

// --- EEPROM save prompt state ---
bool awaitingEepromConfirm = false;
unsigned long eepromPromptTime = 0;
unsigned long eepromReHoldStart = 0;
bool eepromReHolding = false;

// --- Save-flow / Load-flow screen state ---
const int SAVE_SCREEN_NONE = 0;
const int SAVE_SCREEN_MAKING_SOUND = 1;
const int SAVE_SCREEN_SOUND_MADE = 2;
const int SAVE_SCREEN_PROMPT = 3;
const int SAVE_SCREEN_RELEASE_TO_CONFIRM = 4;
const int SAVE_SCREEN_HOLD_TO_CONFIRM = 5;
const int SAVE_SCREEN_SOUND_SAVED = 6;
const int SAVE_SCREEN_SAVE_CANCELLED = 7;
const int SAVE_SCREEN_LOADING = 8;
const int SAVE_SCREEN_LOADED = 9;

int saveScreenState = SAVE_SCREEN_NONE;
unsigned long saveScreenShownAt = 0;
const unsigned long BRIEF_SAVE_SCREEN_DURATION_MS = 1750;

const unsigned long SAVE_SCREEN_GRACE_PERIOD_MS = 800;

// --- Recording (glide-aware event engine) ---
// Instead of sampling at a fixed rate regardless of activity, a held
// steady note costs a single stored event, and only genuine pitch
// movement (a "glide") is sampled continuously. Detection works per
// touched string: a brief settle window after touch-down lets the
// anchor re-pin freely (absorbing the mechanical settling of a finger
// landing on the string), then any drift beyond a percentage of the
// anchor pitch (see glideJitterThresholdForFreq()) is treated as the
// start of a deliberate glide -- once triggered, full-rate sampling
// continues for the rest of that press regardless of whether movement
// pauses again, and only stops on physical release. This keeps
// continuous pitch movement (the character that made the old
// fixed-rate sampler feel "exact") while no longer paying full
// sampling cost for long held notes or silence.
#define GLIDE_SAMPLE_INTERVAL_MS 80
#define MAX_EXACT_EVENTS 300
#define LOOP_PAD 60
struct Sample {
  uint16_t freq;
  // Milliseconds since the start of the recording (or, after loading,
  // since exactLoopStartIndex) -- NOT a fixed-rate tick count. Events
  // are written at variable, non-tick-aligned times under the
  // glide-aware engine, so storing a true millisecond offset directly
  // (rather than elapsed/GLIDE_SAMPLE_INTERVAL_MS as the old fixed-rate
  // sampler did) is required for onset/release timing to stay precise.
  // uint16_t caps this at 65535ms (~65.5s) per loop, comfortably above
  // this project's actual loop durations.
  uint16_t timeIndex;
};
Sample exactRecorded[MAX_EXACT_EVENTS];
int exactEventCount = 0;
unsigned long exactRecordStartTime = 0;
unsigned long exactLoopDuration = 0;
uint16_t exactLoopStartIndex = 0;
unsigned long lastGlideSampleTime = 0;

// --- Glide detection state (per string, mode 4 recording only) ---
// Threshold is a percentage of the current anchor pitch, not a flat Hz
// value -- a flat Hz threshold is a much bigger fraction of a low
// string's pitch than a high string's (e.g. 6Hz was ~12% of a ~49Hz
// bass note but only ~0.3% of a ~1976Hz high note), so sensitivity felt
// wildly inconsistent across the instrument's range. GLIDE_JITTER_
// THRESHOLD_FLOOR_HZ keeps the threshold from collapsing toward 0 at
// very low pitches, where 1.5% alone would truncate to 0-1Hz under
// integer math and make the engine trigger on ordinary sensor noise
// instead of real, deliberate movement.
const int GLIDE_JITTER_THRESHOLD_PERCENT_X10 = 15; // 1.5%, x10 to stay in integer math
const int GLIDE_JITTER_THRESHOLD_FLOOR_HZ = 2;
const unsigned long GLIDE_TOUCHDOWN_SETTLE_MS = 60;
int glideAnchorFreq[4] = {0, 0, 0, 0};
bool isGliding[4] = {false, false, false, false};
unsigned long touchDownTime[4] = {0, 0, 0, 0};
bool exactRecordOverflowed = false;

// Returns the glide-detection drift threshold, in Hz, for a given anchor
// pitch -- GLIDE_JITTER_THRESHOLD_PERCENT_X10 percent of anchorFreq,
// never below GLIDE_JITTER_THRESHOLD_FLOOR_HZ. Pure integer math (no
// floats) since this sits on a per-loop-iteration hot path.
int glideJitterThresholdForFreq(int anchorFreq) {
  int pctThreshold = (anchorFreq * GLIDE_JITTER_THRESHOLD_PERCENT_X10) / 1000;
  return max(pctThreshold, GLIDE_JITTER_THRESHOLD_FLOOR_HZ);
}

// --- Exact-recording capacity warning ---
const int MAX_EXACT_EVENTS_WARN_THRESHOLD = (MAX_EXACT_EVENTS * 8) / 10;
bool exactCapacityWarningGiven = false;
// Set the instant the 80% warning fires; drives a 4-second flash of the
// Record screen's capacity marker line. -1 means "no flash active."
long capacityWarningFlashStartTime = -1;
const unsigned long CAPACITY_WARNING_FLASH_DURATION_MS = 4000;
const unsigned long CAPACITY_WARNING_FLASH_HALF_PERIOD_MS = 300;

// --- Last touched string tracking ---
int lastActiveString = -1;
int prevPot[4] = {0, 0, 0, 0};

// --- Speed ---
unsigned long playbackStartTime = 0;
int exactSpeedPercent = 100;
const int EXACT_SPEED_MAX = 800;
const int EXACT_SPEED_MIN = 25;

// --- Pitch shift ---
int pitchShiftPercent = 100;

// --- Fade ---
int fadeAmountPercent = 0;
int savedFadeAmountPercent = 0;

// --- Sound mode ---
unsigned long soundTriggerStart[4] = {0, 0, 0, 0};
bool soundTriggerActive[4] = {false, false, false, false};
bool soundTriggerReleased[4] = {false, false, false, false};
unsigned long soundTriggerReleaseTime[4] = {0, 0, 0, 0};
int soundTriggerPitchPercent[4] = {100, 100, 100, 100};

int activeStringForDisplay = -1;

const int stringDisplayBoxOrder[4] = {2, 3, 1, 0};

// --- Encoder turn flash ---
unsigned long lastEncoderChangeTime = 0;
bool lastEncoderChangeWasClockwise = true;
const unsigned long ENCODER_FLASH_DURATION_MS = 250;

// --- Display refresh throttle ---
const unsigned long DISPLAY_FRAME_INTERVAL_MS = 1000 / 30;
unsigned long lastDisplayFrameTime = 0;

// --- Playback Effects: Tremolo, Vibrato, Bitcrusher (modes 6/7/8) ---
// Each effect has a single 0-100 "character" knob, plus an on/off flag
// toggled by a single click. The on/off state PERSISTS across cycling
// between modes, AND applies continuously in the background to
// whatever is currently playing (loop playback or Sound-mode single
// voice), regardless of which mode screen is currently displayed.
// Adjusting a knob or toggling on/off still requires being on that
// effect's own screen; only exitReplayMode() resets all three off.

int tremoloKnob = 50;     // 0-100, see tremoloRateFromKnob()/tremoloDepthFromKnob()
bool tremoloOn = false;

int vibratoKnob = 50;     // 0-100, see vibratoRateFromKnob()/vibratoDepthFromKnob()
bool vibratoOn = false;

int bitcrushKnob = 50;    // 0-100, see bitcrushCoarsenessFromKnob()/bitcrushHarshnessFromKnob()
bool bitcrushOn = false;

const int EFFECT_KNOB_MIN = 0;
const int EFFECT_KNOB_MAX = 100;
const int EFFECT_KNOB_STEP = 5; // amount one encoder tick moves a knob

// Tremolo: a slow volume oscillation (an amplitude wobble). Knob=0 is
// slow and shallow, knob=100 is fast and deep.
const float TREMOLO_RATE_MIN = 1.0;   // Hz -- oscillations per second, at knob=0
const float TREMOLO_RATE_MAX = 15.0;  // Hz, at knob=100
const int TREMOLO_DEPTH_MIN = 10;     // percent volume swing, at knob=100 (deepest)
const int TREMOLO_DEPTH_MAX = 90;     // percent volume swing, at knob=0 (shallowest)

// Vibrato: a pitch oscillation. Knob=0 is slow and shallow, knob=100 is
// fast and deep -- same inverse-mapping idea as Tremolo above.
const float VIBRATO_RATE_MIN = 1.0;   // Hz, at knob=0
const float VIBRATO_RATE_MAX = 15.0;  // Hz, at knob=100
const float VIBRATO_DEPTH_MIN = 1.0;  // percent pitch swing, at knob=100 (deepest)
const float VIBRATO_DEPTH_MAX = 15.0; // percent pitch swing, at knob=0 (shallowest)

// Bitcrusher: degrades the signal two ways at once. "Coarseness" holds
// the played frequency static for a stretch of time instead of letting
// it update continuously (a lo-fi sample-and-hold effect). "Harshness"
// periodically gates the volume to silence within a fixed 60ms cycle
// (see applyBitcrushVolume()), like a fast on/off chop.
const int BITCRUSH_COARSENESS_MIN = 20;  // ms held per step, at knob=100 (smoothest)
const int BITCRUSH_COARSENESS_MAX = 200; // ms held per step, at knob=0 (choppiest)
const int BITCRUSH_HARSHNESS_MIN = 10;   // percent of each 60ms cycle silenced, at knob=0
const int BITCRUSH_HARSHNESS_MAX = 90;   // percent of each 60ms cycle silenced, at knob=100

int bitcrushHeldFreq = 0;
unsigned long bitcrushLastStepTime = 0;

int bitcrushStaticFrameIndex = 0;
unsigned long bitcrushStaticLastFlipTime = 0;

// PITCH_BASELINE/FADE_BASELINE are fixed "off" values for their modes.
// EXACT_SPEED's baseline isn't fixed -- it's whatever speed was active
// when first leaving Record, captured once into exactSpeedBaseline (see
// exactSpeedBaselineCaptured below). A single click in Speed/Pitch/Fade
// snaps the current value to its baseline and remembers what it was;
// clicking again restores the remembered value. *RememberedValue == -1
// means "nothing to restore yet."
const int PITCH_BASELINE = 100;
const int FADE_BASELINE = 0;

int exactSpeedBaseline = 100;
bool exactSpeedBaselineCaptured = false;
int exactSpeedRememberedValue = -1;

int pitchRememberedValue = -1;
int fadeRememberedValue = -1;

// --- EEPROM layout constants ---
// NOTE: this layout removes the old type byte (quantized vs exact no
// longer exists) and the old BPM address slot. This is a breaking
// change to the save format -- any save written under a prior layout
// will read back as garbled data the first time it's loaded after this
// change. A fresh save under this layout resolves it going forward.
#define EEPROM_MAGIC 0xA5
#define EEPROM_HEADER_SIZE 22
#define EEPROM_DATA_START 22
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_COUNT_ADDR 2
#define EEPROM_PITCH_ADDR 4
#define EEPROM_FADE_ADDR 6
#define EEPROM_SPEED_ADDR 8
#define EEPROM_LOOPSTART_ADDR 10
#define EEPROM_TREMOLO_ON_ADDR 12
#define EEPROM_VIBRATO_ON_ADDR 13
#define EEPROM_BITCRUSH_ON_ADDR 14
#define EEPROM_TREMOLO_KNOB_ADDR 16
#define EEPROM_VIBRATO_KNOB_ADDR 18
#define EEPROM_BITCRUSH_KNOB_ADDR 20

// --- Tuning Presets [preset][string] = {low, high} ---
const int tunings[6][4][2] = {
  { {NOTE_G2, NOTE_D4}, {NOTE_D2, NOTE_A3}, {NOTE_A1, NOTE_E3}, {NOTE_E1, NOTE_B2} },
  { {NOTE_G1, NOTE_D3}, {NOTE_D1, NOTE_A2}, {NOTE_G3, NOTE_B6}, {NOTE_G3, NOTE_B6} },
  { {NOTE_G4, NOTE_D6}, {NOTE_C4, NOTE_G5}, {NOTE_E4, NOTE_B5}, {NOTE_A4, NOTE_E6} },
  { {NOTE_G3, NOTE_D5}, {NOTE_C3, NOTE_G4}, {NOTE_E3, NOTE_B4}, {NOTE_A3, NOTE_E5} },
  { {NOTE_G3, NOTE_D5}, {NOTE_D4, NOTE_A5}, {NOTE_A4, NOTE_E6}, {NOTE_E5, NOTE_B6} },
  { {NOTE_G3, NOTE_B3}, {NOTE_A3, NOTE_C4}, {NOTE_B3, NOTE_D4}, {NOTE_C4, NOTE_E4} }
};

// ===================== CHIME LIBRARY =====================
// Every chime here is a blocking sequence of toneAC() + delay() calls --
// the display does not update and input is not read while one plays.
// Kept short for that reason. Most are heard at a specific transition;
// see the comment above each for exactly when.

// Generic failure cue: no string touched while loading from EEPROM,
// or the event buffer is already full and a new event was refused.
void playErrorTone() {
  toneAC(NOTE_A3, 10, 150, false); delay(180);
  toneAC(NOTE_A2, 10, 150, false); delay(180);
  noToneAC();
}

// Played once, leaving Record for Playback Speed after a successful
// recording (single click in Record mode with at least one event).
void playPlaybackEntryChime() {
  toneAC(NOTE_E5, 10, 120, false); delay(140);
  noToneAC();
}

// Played entering Record mode from Tuning (single click).
void playExactRecordChime() {
  toneAC(NOTE_G5, 10, 80, false); delay(100);
  toneAC(NOTE_B5, 10, 80, false); delay(100);
  toneAC(NOTE_D6, 10, 110, false); delay(130);
  noToneAC();
}

// Brief "heads up" cue at 80% capacity during exact recording --
// informational, not a failure, so it's pitched up an octave from
// playErrorTone() rather than reusing its register, but otherwise the
// same falling two-note shape. Full volume (10), not 9 -- toneAC's
// volume control is linear, but perceived loudness is not, so one step
// down from the top is the single biggest perceptual drop on the whole
// 0-10 scale. That's almost certainly why this was hard to hear before.
void playExactCapacityWarningChime() {
  toneAC(NOTE_A4, 10, 150, false); delay(180);
  toneAC(NOTE_A3, 10, 150, false); delay(180);
  noToneAC();
}

// Recording was cut short because the event buffer filled mid-glide.
// This is a worse outcome than an ordinary "max reached" stop (some of
// what was played is lost), so it deliberately sounds more like a
// failure: a lower, starker variant of playErrorTone()'s falling
// two-note shape rather than reusing it outright.
void playExactRecordOverflowChime() {
  toneAC(NOTE_A2, 10, 160, false); delay(190);
  toneAC(NOTE_E2, 10, 200, false); delay(230);
  noToneAC();
}

// Played entering Playback Pitch (mode 3), via double-click.
void playPitchModeChime() {
  toneAC(NOTE_C6, 10, 70, false); delay(90);
  toneAC(NOTE_F6, 10, 90, false); delay(110);
  noToneAC();
}

// Played entering Playback Speed (mode 2) via double-click, wrapping
// the effect chain around from Bitcrusher back to the start.
void playSpeedModeChime() {
  toneAC(NOTE_C4, 10, 70, false); delay(90);
  toneAC(NOTE_E4, 10, 90, false); delay(110);
  noToneAC();
}

// Shared by every double-click transition into Fade, Tremolo, Vibrato,
// and Bitcrusher (modes 4/6/7/8) -- the name is historical (Fade was
// the first of these modes), the sound just means "moved forward one
// step in the effect chain."
void playFadeModeChime() {
  toneAC(NOTE_E5, 10, 70, false); delay(90);
  toneAC(NOTE_C5, 10, 70, false); delay(90);
  toneAC(NOTE_G4, 10, 90, false); delay(110);
  noToneAC();
}

// Played whenever exitReplayMode() runs -- the spam-click exit gesture,
// from any non-Tuning mode.
void playExitToTuningChime() {
  toneAC(NOTE_G5, 10, 90, false); delay(110);
  toneAC(NOTE_C5, 10, 110, false); delay(130);
  noToneAC();
}

// Played when a Sound is created (hold gesture in any playback/effect
// mode), confirming the current loop + pitch + fade were frozen for
// per-string triggering.
void playSoundSavedChime() {
  toneAC(NOTE_C5, 10, 70, false); delay(85);
  toneAC(NOTE_E5, 10, 70, false); delay(85);
  toneAC(NOTE_G5, 10, 70, false); delay(85);
  toneAC(NOTE_C6, 10, 120, false); delay(140);
  noToneAC();
}

// Played when the hold-to-save gesture first triggers in Sound mode,
// asking the player to release and re-hold to confirm.
void playEepromPromptChime() {
  toneAC(NOTE_A4, 10, 80, false); delay(95);
  toneAC(NOTE_E4, 10, 80, false); delay(95);
  toneAC(NOTE_A4, 10, 110, false); delay(130);
  noToneAC();
}

// Played once the re-hold confirms and the loop is actually written to
// EEPROM.
void playEepromConfirmedChime() {
  toneAC(NOTE_G4, 10, 80, false); delay(95);
  toneAC(NOTE_C5, 10, 80, false); delay(95);
  toneAC(NOTE_E5, 10, 80, false); delay(95);
  toneAC(NOTE_C6, 10, 160, false); delay(190);
  noToneAC();
}

// Played if the save prompt times out (eepromAbortTimeout) without a
// confirming re-hold.
void playEepromAbortedChime() {
  toneAC(NOTE_D5, 10, 80, false); delay(95);
  toneAC(NOTE_A4, 10, 80, false); delay(95);
  toneAC(NOTE_D4, 10, 100, false); delay(120);
  noToneAC();
}

// Played after a successful hold-to-load from Tuning mode.
void playEepromLoadedChime() {
  toneAC(NOTE_E5, 10, 75, false); delay(90);
  toneAC(NOTE_A5, 10, 75, false); delay(90);
  toneAC(NOTE_D6, 10, 130, false); delay(150);
  noToneAC();
}

// ===================== EEPROM SAVE / LOAD =====================
// One loop "slot" total -- saving always overwrites whatever was there.
// See the EEPROM layout constants above for the exact byte addresses;
// EEPROM_MAGIC is the first byte written on save but the first byte
// checked on load, so a save interrupted partway through (power loss,
// reset) is far more likely to leave the magic byte correct but the
// data after it garbled than the reverse -- loadLoopFromEeprom() has
// no way to detect that case today (see Section 4 / sentinel-check
// open item if extending this).

void saveLoopToEeprom() {
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_COUNT_ADDR, (uint16_t)exactEventCount);
  EEPROM.put(EEPROM_PITCH_ADDR, (uint16_t)pitchShiftPercent);
  EEPROM.put(EEPROM_FADE_ADDR, (uint16_t)savedFadeAmountPercent);
  EEPROM.put(EEPROM_SPEED_ADDR, (uint16_t)exactSpeedPercent);
  EEPROM.put(EEPROM_LOOPSTART_ADDR, (uint16_t)exactLoopStartIndex);
  EEPROM.write(EEPROM_TREMOLO_ON_ADDR, tremoloOn ? 1 : 0);
  EEPROM.write(EEPROM_VIBRATO_ON_ADDR, vibratoOn ? 1 : 0);
  EEPROM.write(EEPROM_BITCRUSH_ON_ADDR, bitcrushOn ? 1 : 0);
  EEPROM.put(EEPROM_TREMOLO_KNOB_ADDR, (uint16_t)tremoloKnob);
  EEPROM.put(EEPROM_VIBRATO_KNOB_ADDR, (uint16_t)vibratoKnob);
  EEPROM.put(EEPROM_BITCRUSH_KNOB_ADDR, (uint16_t)bitcrushKnob);
  int addr = EEPROM_DATA_START;
  // addr + 4 <= EEPROM.length() stops writing before running past the
  // physical chip's actual size, rather than relying on exactEventCount
  // alone never exceeding what fits.
  for (int i = 0; i < exactEventCount && addr + 4 <= EEPROM.length(); i++) {
    EEPROM.put(addr, exactRecorded[i].freq); addr += 2;
    EEPROM.put(addr, exactRecorded[i].timeIndex); addr += 2;
  }
  Serial.println("Saved loop to EEPROM.");
}

// Same job as computeExactLoopBoundaries(), but for a loop that just
// came from EEPROM rather than a live recording: exactLoopStartIndex
// is already known (it was saved/loaded directly), so this only needs
// to derive the duration from it -- no first/last-active-event scan or
// LOOP_PAD trimming, since that trimming already happened once, before
// the original save.
void computeExactLoopBoundariesForLoad() {
  if (exactEventCount == 0) { exactLoopDuration = 0; return; }
  unsigned long endMs = (unsigned long)exactRecorded[exactEventCount - 1].timeIndex + 1;
  unsigned long startMs = (unsigned long)exactLoopStartIndex;
  exactLoopDuration = (endMs > startMs) ? (endMs - startMs) : 1;
}

// Returns false (and changes nothing) if no valid save exists yet --
// callers should check the return value rather than assume a loop was
// loaded just because this function ran.
bool loadLoopFromEeprom() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) return false;
  uint16_t count;
  EEPROM.get(EEPROM_COUNT_ADDR, count);
  uint16_t savedPitch, savedFade, savedSpeed, savedLoopStart;
  EEPROM.get(EEPROM_PITCH_ADDR, savedPitch);
  EEPROM.get(EEPROM_FADE_ADDR, savedFade);
  EEPROM.get(EEPROM_SPEED_ADDR, savedSpeed);
  EEPROM.get(EEPROM_LOOPSTART_ADDR, savedLoopStart);
  pitchShiftPercent = savedPitch;
  fadeAmountPercent = savedFade;
  savedFadeAmountPercent = savedFade;
  exactSpeedPercent = savedSpeed;
  exactLoopStartIndex = savedLoopStart;

  tremoloOn = (EEPROM.read(EEPROM_TREMOLO_ON_ADDR) == 1);
  vibratoOn = (EEPROM.read(EEPROM_VIBRATO_ON_ADDR) == 1);
  bitcrushOn = (EEPROM.read(EEPROM_BITCRUSH_ON_ADDR) == 1);
  uint16_t savedTremoloKnob, savedVibratoKnob, savedBitcrushKnob;
  EEPROM.get(EEPROM_TREMOLO_KNOB_ADDR, savedTremoloKnob);
  EEPROM.get(EEPROM_VIBRATO_KNOB_ADDR, savedVibratoKnob);
  EEPROM.get(EEPROM_BITCRUSH_KNOB_ADDR, savedBitcrushKnob);
  // constrain() here guards against a future layout change leaving
  // stale/out-of-range bytes at these addresses, the same class of
  // problem the breaking-change history above already describes.
  tremoloKnob = constrain((int)savedTremoloKnob, EFFECT_KNOB_MIN, EFFECT_KNOB_MAX);
  vibratoKnob = constrain((int)savedVibratoKnob, EFFECT_KNOB_MIN, EFFECT_KNOB_MAX);
  bitcrushKnob = constrain((int)savedBitcrushKnob, EFFECT_KNOB_MIN, EFFECT_KNOB_MAX);

  int addr = EEPROM_DATA_START;
  exactEventCount = min((int)count, MAX_EXACT_EVENTS);
  for (int i = 0; i < exactEventCount; i++) {
    uint16_t f, t;
    EEPROM.get(addr, f); addr += 2;
    EEPROM.get(addr, t); addr += 2;
    exactRecorded[i].freq = f;
    exactRecorded[i].timeIndex = t;
  }
  computeExactLoopBoundariesForLoad();
  Serial.println("Loaded loop from EEPROM.");
  return true;
}

void exitReplayMode() {
  noToneAC();
  playExitToTuningChime();
  lastActiveString = -1;
  pitchShiftPercent = 100;
  exactSpeedPercent = 100;
  exactLoopDuration = 0;
  exactLoopStartIndex = 0;
  exactEventCount = 0;
  exactRecordOverflowed = false;
  exactCapacityWarningGiven = false;
  capacityWarningFlashStartTime = -1;
  for (int i = 0; i < 4; i++) {
    isGliding[i] = false;
    glideAnchorFreq[i] = 0;
    touchDownTime[i] = 0;
  }
  fadeAmountPercent = 0;
  savedFadeAmountPercent = 0;
  awaitingEepromConfirm = false;
  eepromReHolding = false;
  spamClickCount = 0;
  loadHoldFired = false;
  inputLockedAfterLoad = false;
  activeStringForDisplay = -1;
  saveScreenState = SAVE_SCREEN_NONE;
  tremoloOn = false;
  vibratoOn = false;
  bitcrushOn = false;
  pitchRememberedValue = -1;
  fadeRememberedValue = -1;
  exactSpeedBaselineCaptured = false;
  exactSpeedRememberedValue = -1;
  for (int i = 0; i < 4; i++) {
    soundTriggerActive[i] = false;
    soundTriggerReleased[i] = false;
  }
  mode = 0;
  exitedAtTime = millis();
  Serial.println("Exited to Tuning mode.");
}

// Commits one event to exactRecorded[], handling the capacity warning
// and overflow cases. Returns false if the buffer is already full (the
// event was NOT written) so callers can react -- e.g. ending the
// recording with the overflow chime if this happened mid-glide.
bool commitExactEvent(uint16_t freq, unsigned long elapsedMs) {
  if (exactEventCount >= MAX_EXACT_EVENTS) {
    return false;
  }
  exactRecorded[exactEventCount].freq = freq;
  exactRecorded[exactEventCount].timeIndex = (uint16_t)min(elapsedMs, (unsigned long)65535);
  exactEventCount++;
  if (!exactCapacityWarningGiven && exactEventCount >= MAX_EXACT_EVENTS_WARN_THRESHOLD) {
    exactCapacityWarningGiven = true;
    capacityWarningFlashStartTime = (long)millis();
    playExactCapacityWarningChime();
  }
  return true;
}

// Finds the actual sounding portion of a fresh recording and trims the
// loop to it, padded by LOOP_PAD ms on each side so a loop doesn't cut
// off right at a note's edge. "Sounding portion" means the span from
// the first to the last event with freq > 0 -- release markers
// (freq == 0) bookend that span but aren't part of it. If every event
// turns out to be a release marker (nothing was ever actually played),
// falls back to using the whole recording as-is rather than leaving
// exactLoopDuration at zero.
void computeExactLoopBoundaries() {
  if (exactEventCount == 0) return;
  int firstActive = -1;
  int lastActive = -1;
  for (int i = 0; i < exactEventCount; i++) {
    if (exactRecorded[i].freq > 0) {
      if (firstActive == -1) firstActive = i;
      lastActive = i;
    }
  }
  if (firstActive == -1) {
    exactLoopStartIndex = 0;
    exactLoopDuration = (unsigned long)exactRecorded[exactEventCount - 1].timeIndex + 1;
    return;
  }
  unsigned long startMs = (unsigned long)exactRecorded[firstActive].timeIndex;
  unsigned long endMs   = (unsigned long)exactRecorded[lastActive].timeIndex + 1;
  unsigned long paddedStart = (startMs > LOOP_PAD) ? (startMs - LOOP_PAD) : 0;
  unsigned long paddedEnd   = endMs + LOOP_PAD;
  exactLoopStartIndex = (uint16_t)paddedStart;
  exactLoopDuration   = paddedEnd - paddedStart;
  Serial.print("Trimmed start ms: ");  Serial.println(paddedStart);
  Serial.print("Trimmed end ms: ");    Serial.println(paddedEnd);
  Serial.print("Loop duration ms: ");  Serial.println(exactLoopDuration);
}

// percent=100 means unchanged pitch; below 100 lowers it, above 100
// raises it. Clamped to a sane audible range (20Hz-20kHz) regardless
// of input, since pitch/speed math elsewhere can in principle push a
// frequency outside what's meaningful to play.
int applyPitchShiftPercent(int freq, int percent) {
  return constrain((int)((long)freq * percent / 100), 20, 20000);
}

// Convenience wrapper for the common case: shift by whatever the
// Playback Pitch mode (mode 3) currently has dialed in.
int applyPitchShift(int freq) {
  return applyPitchShiftPercent(freq, pitchShiftPercent);
}

// toneAC's volume parameter is 0-10, not 0-100 -- this returns a value
// already in that range. fadeAmt is "how far the volume falls over one
// full loop": at fadeAmt=100 volume ramps linearly from 10 (loop start)
// down to 0 (loop end); at fadeAmt=50 it only ramps down to 5; at
// fadeAmt=0 (or loopLen=0) fade is off entirely and this just returns
// full volume (10).
int computeFadeVolumeWithAmount(unsigned long playPos, unsigned long loopLen, int fadeAmt) {
  if (fadeAmt <= 0 || loopLen == 0) return 10;
  long minVolume10x = 100 - fadeAmt;
  long progress1000 = ((long)playPos * 1000) / loopLen;
  long volume10x = 100 - ((100 - minVolume10x) * progress1000 / 1000);
  int volume = (int)(volume10x / 10);
  return constrain(volume, 0, 10);
}

// Convenience wrapper using whatever Playback Fade mode (mode 4)
// currently has dialed in.
int computeFadeVolume(unsigned long playPos, unsigned long loopLen) {
  return computeFadeVolumeWithAmount(playPos, loopLen, fadeAmountPercent);
}

// --- Effect parameter derivation: each knob (0-100) maps inversely to
// two underlying values. knob=0 is one extreme, knob=100 is the other.

// Hz -- how many times per second the volume oscillates.
float tremoloRateFromKnob() {
  return TREMOLO_RATE_MIN + (tremoloKnob / 100.0) * (TREMOLO_RATE_MAX - TREMOLO_RATE_MIN);
}
// percent -- how far the volume swings from its base value each cycle.
int tremoloDepthFromKnob() {
  return TREMOLO_DEPTH_MAX - (int)((tremoloKnob / 100.0) * (TREMOLO_DEPTH_MAX - TREMOLO_DEPTH_MIN));
}

// Hz -- how many times per second the pitch oscillates.
float vibratoRateFromKnob() {
  return VIBRATO_RATE_MIN + (vibratoKnob / 100.0) * (VIBRATO_RATE_MAX - VIBRATO_RATE_MIN);
}
// percent -- how far the pitch swings from its base value each cycle.
float vibratoDepthFromKnob() {
  return VIBRATO_DEPTH_MAX - (vibratoKnob / 100.0) * (VIBRATO_DEPTH_MAX - VIBRATO_DEPTH_MIN);
}

// Milliseconds the played frequency is held static before it's allowed
// to update again -- the "sample and hold" half of the bitcrush effect.
int bitcrushCoarsenessFromKnob() {
  return BITCRUSH_COARSENESS_MAX - (int)((bitcrushKnob / 100.0) * (BITCRUSH_COARSENESS_MAX - BITCRUSH_COARSENESS_MIN));
}
// Percent of each fixed-length gate cycle (see applyBitcrushVolume's
// gatePeriodMs) that gets silenced -- the "chop" half of the effect.
int bitcrushHarshnessFromKnob() {
  return BITCRUSH_HARSHNESS_MIN + (int)((bitcrushKnob / 100.0) * (BITCRUSH_HARSHNESS_MAX - BITCRUSH_HARSHNESS_MIN));
}

int applyTremoloVolume(int baseVolume, unsigned long now) {
  float rate = tremoloRateFromKnob();
  int depth = tremoloDepthFromKnob();
  float phase = (now / 1000.0) * rate * 2 * PI;
  float oscillator = (sin(phase) + 1.0) / 2.0;
  float minVolumeFactor = 1.0 - (depth / 100.0);
  float volumeFactor = minVolumeFactor + (1.0 - minVolumeFactor) * oscillator;
  return constrain((int)(baseVolume * volumeFactor), 0, 10);
}

int applyVibratoFrequency(int baseFreq, unsigned long now) {
  float rate = vibratoRateFromKnob();
  float depth = vibratoDepthFromKnob();
  float phase = (now / 1000.0) * rate * 2 * PI;
  float oscillator = sin(phase);
  float freqMultiplier = 1.0 + (oscillator * depth / 100.0);
  return constrain((int)(baseFreq * freqMultiplier), 20, 20000);
}

// Sample-and-hold: keeps returning the same frequency until coarseness
// ms have passed since the last update, then locks onto whatever
// baseFreq is at that moment. Higher coarseness (lower knob) = longer
// holds = choppier, more obviously stepped pitch.
int applyBitcrushFrequency(int baseFreq, unsigned long now) {
  int coarseness = bitcrushCoarsenessFromKnob();
  if (now - bitcrushLastStepTime >= (unsigned long)coarseness) {
    bitcrushHeldFreq = baseFreq;
    bitcrushLastStepTime = now;
  }
  return bitcrushHeldFreq;
}

// Repeating 60ms gate: silences the signal for the first harshness%
// of every cycle, then lets it through for the rest. Independent of
// applyBitcrushFrequency() above -- one chops volume, the other chops
// pitch resolution, and bitcrushOn applies both (see runExactPlayback).
int applyBitcrushVolume(int baseVolume, unsigned long now) {
  int harshness = bitcrushHarshnessFromKnob();
  const float gatePeriodMs = 60.0;
  float cyclePosition = fmod(now, gatePeriodMs) / gatePeriodMs;
  float onThreshold = harshness / 100.0;
  return (cyclePosition < onThreshold) ? 0 : baseVolume;
}

// Drives loop playback for modes 2/3/4/6/7/8 -- all six share this one
// function since they're all just viewing/adjusting the same underlying
// loop with different controls. The recorded events in exactRecorded[]
// carry real timestamps from when the loop was captured (always at
// normal speed); scaledLoopLen/scaledPos translate between "wall clock
// time at the current playback speed" and "the original recording's
// timeline" so the lookup below finds the right event regardless of
// exactSpeedPercent. At 200% speed the loop plays in half the wall-clock
// time; at 50% it takes twice as long -- scaledLoopLen reflects that,
// while scaledPos converts a position within it back to where that
// moment actually falls in the original (100%-speed) recording.
void runExactPlayback(unsigned long now) {
  if (exactEventCount == 0 || exactLoopDuration == 0) return;
  unsigned long scaledLoopLen = (unsigned long)((unsigned long long)exactLoopDuration * 100 / exactSpeedPercent);
  unsigned long playPos = (now - playbackStartTime) % scaledLoopLen;
  unsigned long scaledPos = (unsigned long)((unsigned long long)playPos * exactSpeedPercent / 100);
  unsigned long absolutePos = scaledPos + (unsigned long)exactLoopStartIndex;
  uint16_t targetIndex = (uint16_t)absolutePos;
  int found = -1;
  // Backward scan for "last event at or before targetIndex" -- correct
  // regardless of how unevenly events are spaced (a held note may have
  // gone hundreds of ms between events; a glide may have one every
  // GLIDE_SAMPLE_INTERVAL_MS), since this doesn't assume any fixed
  // spacing at all.
  for (int i = exactEventCount - 1; i >= 0; i--) {
    if (exactRecorded[i].timeIndex <= targetIndex) { found = i; break; }
  }
  if (found >= 0 && exactRecorded[found].freq > 0) {
    int freq = constrain(applyPitchShift(exactRecorded[found].freq), 20, 20000);
    int vol = computeFadeVolume(playPos, scaledLoopLen);

    // Fixed application order: bitcrush's frequency hold, then vibrato's
    // wobble, then tremolo's volume swell, then bitcrush's volume gate
    // last. Matches runSoundStringExact() below exactly, so a Sound
    // triggered from this loop sounds the same as the loop itself.
    if (bitcrushOn) {
      freq = applyBitcrushFrequency(freq, now);
    }
    if (vibratoOn) {
      freq = applyVibratoFrequency(freq, now);
    }
    if (tremoloOn) {
      vol = applyTremoloVolume(vol, now);
    }
    if (bitcrushOn) {
      vol = applyBitcrushVolume(vol, now);
    }

    freq = constrain(freq, 20, 20000);
    vol = constrain(vol, 0, 10);

    if (vol <= 0) noToneAC();
    else toneAC(freq, vol, 0, true);
  } else {
    noToneAC();
  }
}

// Freezes the current loop + pitch + fade for per-string Sound-mode
// triggering. Doesn't touch exactRecorded[]/exactLoopDuration at all --
// those already hold what gets frozen. Just snapshots the fade amount
// (Sound mode has its own fade behavior via savedFadeAmountPercent,
// separate from live Playback Fade) and clears any stale trigger state
// left over from a previous Sound.
void saveCurrentAsSound() {
  savedFadeAmountPercent = fadeAmountPercent;
  for (int i = 0; i < 4; i++) {
    soundTriggerActive[i] = false;
    soundTriggerReleased[i] = false;
  }
  playSoundSavedChime();
}

// Sound mode's per-string equivalent of runExactPlayback() -- plays the
// frozen loop once from its own start (soundTriggerStart[stringIndex])
// rather than looping continuously, and applies a short linear fade-out
// after release (releaseFadeOutMs) instead of cutting off instantly.
// Per-string pitch (soundTriggerPitchPercent) comes from where the
// string is currently pressed, not from the global Playback Pitch mode.
void runSoundStringExact(int stringIndex, unsigned long now) {
  if (exactEventCount == 0 || exactLoopDuration == 0) return;
  unsigned long elapsed = now - soundTriggerStart[stringIndex];
  if (elapsed >= exactLoopDuration) {
    soundTriggerActive[stringIndex] = false;
    soundTriggerReleased[stringIndex] = false;
    return;
  }
  unsigned long absolutePos = elapsed + (unsigned long)exactLoopStartIndex;
  uint16_t targetIndex = (uint16_t)absolutePos;
  int found = -1;
  for (int i = exactEventCount - 1; i >= 0; i--) {
    if (exactRecorded[i].timeIndex <= targetIndex) { found = i; break; }
  }
  if (found < 0 || exactRecorded[found].freq == 0) return;
  int freqToPlay = applyPitchShiftPercent(exactRecorded[found].freq, soundTriggerPitchPercent[stringIndex]);
  int vol = computeFadeVolumeWithAmount(elapsed, exactLoopDuration, savedFadeAmountPercent);

  if (bitcrushOn) {
    freqToPlay = applyBitcrushFrequency(freqToPlay, now);
  }
  if (vibratoOn) {
    freqToPlay = applyVibratoFrequency(freqToPlay, now);
  }
  if (tremoloOn) {
    vol = applyTremoloVolume(vol, now);
  }
  if (bitcrushOn) {
    vol = applyBitcrushVolume(vol, now);
  }

  // Linear fade-out over releaseFadeOutMs after the string lifts,
  // instead of cutting to silence immediately -- releaseFactor walks
  // from 100 down to 0 over that window and scales vol accordingly.
  if (soundTriggerReleased[stringIndex]) {
    unsigned long sinceRelease = now - soundTriggerReleaseTime[stringIndex];
    if (sinceRelease >= releaseFadeOutMs) {
      soundTriggerActive[stringIndex] = false;
      soundTriggerReleased[stringIndex] = false;
      return;
    }
    long releaseFactor = 100 - ((long)sinceRelease * 100 / releaseFadeOutMs);
    vol = (int)((long)vol * releaseFactor / 100);
  }
  freqToPlay = constrain(freqToPlay, 20, 20000);
  vol = constrain(vol, 0, 10);
  if (vol > 0) toneAC(freqToPlay, vol, 0, true);
}

// --- Encoder smoothing helpers ---

// One encoder tick, already smoothed and direction-resolved by
// processSmoothedTick() below -- dispatches by mode to whichever value
// that mode's screen lets the encoder adjust. Each mode branch is
// independent; only one runs per call. clockwise increases the value,
// counterclockwise decreases it, except Tuning/Sound (mode 0/5) where
// turning instead cycles through tuningNames[] -- there's nothing
// continuous to adjust there, just a preset to step through.
void applyEncoderTick(int direction) {
  bool clockwise = (direction > 0);
  unsigned long now = millis();

  lastEncoderChangeTime = now;
  lastEncoderChangeWasClockwise = clockwise;

  if (mode == 0 || mode == 5) {
    if (!knobActive) {
      if (clockwise) {
        tuningPreset = (tuningPreset + 1) % 6;
      } else {
        tuningPreset = (tuningPreset + 5) % 6;
      }
      knobActive = true;
      Serial.print("Tuning preset: "); Serial.println(tuningPreset);
    }
    lastTurnTime = now;
  } else if (mode == 3) {
    if (clockwise) pitchShiftPercent = min((int)((long)pitchShiftPercent * 106 / 100), 400);
    else           pitchShiftPercent = max((int)((long)pitchShiftPercent * 100 / 106), 25);
    Serial.print("Pitch shift %: "); Serial.println(pitchShiftPercent);
  } else if (mode == 4) {
    if (clockwise) fadeAmountPercent = min(fadeAmountPercent + 5, 100);
    else           fadeAmountPercent = max(fadeAmountPercent - 5, 0);
    Serial.print("Fade amount %: "); Serial.println(fadeAmountPercent);
  } else if (mode == 2) {
    if (clockwise) exactSpeedPercent = min(exactSpeedPercent + 10, EXACT_SPEED_MAX);
    else           exactSpeedPercent = max(exactSpeedPercent - 10, EXACT_SPEED_MIN);
    playbackStartTime = now;
    Serial.print("Exact speed %: "); Serial.println(exactSpeedPercent);
  } else if (mode == 6) {
    if (clockwise) tremoloKnob = min(tremoloKnob + EFFECT_KNOB_STEP, EFFECT_KNOB_MAX);
    else           tremoloKnob = max(tremoloKnob - EFFECT_KNOB_STEP, EFFECT_KNOB_MIN);
    Serial.print("Tremolo knob: "); Serial.println(tremoloKnob);
  } else if (mode == 7) {
    if (clockwise) vibratoKnob = min(vibratoKnob + EFFECT_KNOB_STEP, EFFECT_KNOB_MAX);
    else           vibratoKnob = max(vibratoKnob - EFFECT_KNOB_STEP, EFFECT_KNOB_MIN);
    Serial.print("Vibrato knob: "); Serial.println(vibratoKnob);
  } else if (mode == 8) {
    if (clockwise) bitcrushKnob = min(bitcrushKnob + EFFECT_KNOB_STEP, EFFECT_KNOB_MAX);
    else           bitcrushKnob = max(bitcrushKnob - EFFECT_KNOB_STEP, EFFECT_KNOB_MIN);
    Serial.print("Bitcrush knob: "); Serial.println(bitcrushKnob);
  }
}

// Filters raw encoder ticks before they reach applyEncoderTick(), to
// reject isolated direction glitches without adding noticeable lag to
// genuine turns. A tick that doesn't yet have 2-of-3 recent ticks
// agreeing with it is held as "pending" for one cycle rather than
// applied immediately. If the very next tick agrees, both fire
// together (so a real turn never loses a step, it's delayed by at
// most one tick). If the next tick disagrees instead, the pending one
// is dropped outright (treated as noise), and the new tick is judged
// on its own 2-of-3 standing -- either applied immediately or, if it
// also lacks support yet, it becomes the new pending tick in turn.
void processSmoothedTick(int direction) {
  if (millis() - lastButtonPressTime < CLICK_TURN_SUPPRESS_MS) {
    return;
  }

  int agreeCount = 0;
  for (int i = 0; i < 3; i++) {
    if (recentTickDirections[i] == direction) agreeCount++;
  }

  if (hasPendingTick) {
    if (direction == pendingTickDirection) {
      applyEncoderTick(pendingTickDirection);
      applyEncoderTick(direction);
      hasPendingTick = false;
    } else {
      hasPendingTick = false;
      if (agreeCount >= 2) {
        applyEncoderTick(direction);
      } else {
        pendingTickDirection = direction;
        hasPendingTick = true;
      }
    }
  } else if (agreeCount >= 2) {
    applyEncoderTick(direction);
  } else {
    pendingTickDirection = direction;
    hasPendingTick = true;
  }

  recentTickDirections[2] = recentTickDirections[1];
  recentTickDirections[1] = recentTickDirections[0];
  recentTickDirections[0] = direction;
}

// ===================== ICON DRAWING HELPERS =====================

void drawHomeIcon(int x, int y) {
  display.drawLine(x, y + 6, x + 6, y, SSD1306_WHITE);
  display.drawLine(x + 6, y, x + 12, y + 6, SSD1306_WHITE);
  display.drawRect(x + 2, y + 6, 8, 6, SSD1306_WHITE);
}

// Two-frame blinking record dot, driven by the same iconFrameB pattern
// already used by drawSoundCreateIcon()/drawFolderIcon() (toggled from
// (now / 400) % 2 in the display block) -- no new timer or stored state,
// matching the existing icon-animation convention exactly.
void drawRecordIcon(int x, int y, bool frameB) {
  int r = frameB ? 5 : 3;
  display.fillCircle(x, y, r, SSD1306_WHITE);
}

void drawLoopIcon(int x, int y) {
  display.drawCircle(x + 6, y + 5, 5, SSD1306_WHITE);
  display.drawLine(x + 10, y, x + 11, y + 3, SSD1306_WHITE);
  display.drawLine(x + 11, y + 3, x + 8, y + 2, SSD1306_WHITE);
}

void drawFadeIcon(int x, int y) {
  display.drawLine(x, y + 8, x + 12, y, SSD1306_WHITE);
}

void drawPluckIcon(int x, int y) {
  display.drawLine(x, y + 4, x + 8, y + 4, SSD1306_WHITE);
  display.drawLine(x + 8, y + 4, x + 12, y, SSD1306_WHITE);
  display.fillCircle(x + 12, y, 1, SSD1306_WHITE);
}

void drawSoundCreateIcon(int x, int y, bool frameB) {
  int h = frameB ? 6 : 3;
  display.drawLine(x, y + 4, x + 3, y + 4 - h, SSD1306_WHITE);
  display.drawLine(x + 3, y + 4 - h, x + 6, y + 4 + h, SSD1306_WHITE);
  display.drawLine(x + 6, y + 4 + h, x + 9, y + 4 - h, SSD1306_WHITE);
  display.drawLine(x + 9, y + 4 - h, x + 12, y + 4, SSD1306_WHITE);
}

void drawFolderIcon(int x, int y, bool frameB) {
  display.drawRect(x, y + 3, 14, 8, SSD1306_WHITE);
  if (frameB) {
    display.drawLine(x, y + 3, x + 4, y, SSD1306_WHITE);
    display.drawLine(x + 4, y, x + 9, y, SSD1306_WHITE);
    display.drawLine(x + 9, y, x + 11, y + 3, SSD1306_WHITE);
  } else {
    display.drawLine(x, y + 3, x + 4, y + 1, SSD1306_WHITE);
    display.drawLine(x + 4, y + 1, x + 9, y + 1, SSD1306_WHITE);
    display.drawLine(x + 9, y + 1, x + 11, y + 3, SSD1306_WHITE);
  }
}

// ===================== SAVE-FLOW / LOAD-FLOW SCREEN HELPERS =====================

void drawSaveScreenMessage(const char* line1, const char* line2) {
  display.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;

  display.getTextBounds(line1, 0, 0, &x1, &y1, &w, &h);
  int line1X = (SCREEN_WIDTH - w) / 2;
  int line1Y = (line2 == nullptr) ? (BLUE_TOP + 30) : (BLUE_TOP + 22);
  display.setCursor(line1X, line1Y);
  display.print(line1);

  if (line2 != nullptr) {
    display.getTextBounds(line2, 0, 0, &x1, &y1, &w, &h);
    int line2X = (SCREEN_WIDTH - w) / 2;
    display.setCursor(line2X, BLUE_TOP + 36);
    display.print(line2);
  }
}

void setSaveScreen(int state) {
  saveScreenState = state;
  saveScreenShownAt = millis();
}

void drawOnOffIndicator(bool isOn) {
  display.setTextSize(1);
  display.setCursor(74, 4);
  display.print(isOn ? "ON" : "OFF");
}

// ===================== BOOT SPLASH SEQUENCE =====================
// A ~3 second animated intro shown once at power-on: a flat line grows
// into a wave, the wave grows chaotic and noisy, then calms back down
// into a small clean wave that holds steady while the "TELELELE"
// wordmark appears. A chime plays alongside it (bootChimeSteps below),
// and the display brightness ramps up from near-off rather than
// snapping straight to full. The 6 PHASE_*_END constants are where the
// boot timeline (0.0 to 1.0) crosses from one of these visual stages
// into the next; drawBootFrame() below picks amplitude/frequency/jitter
// for the wave based on which stage the current progress falls in.

const unsigned long CRT_WARMUP_DELAY_MS = 2500; // blank pause before anything draws, evoking an old CRT warming up
const unsigned long BOOT_DURATION_MS = 3000;    // total animated portion, after the warmup pause
const unsigned long BOOT_FRAME_INTERVAL_MS = 1000 / 30; // 30fps target for the boot animation specifically

// Fractions of BOOT_DURATION_MS where the wave's visual character
// changes. In order: still flat line -> growing into a wave -> chaotic
// and noisy -> calming back down -> held steady -> wordmark appears.
const float PHASE_STILL_END     = 0.125f;
const float PHASE_BUILDING_END  = 0.333f;
const float PHASE_CHAOS_END     = 0.457f;
const float PHASE_RESOLVING_END = 0.583f;
const float PHASE_SETTLED_END   = 0.75f;
const float PHASE_WORDMARK_END  = 0.875f;

// One note in the boot chime, with its own absolute start time so the
// notes can be spaced out deliberately rather than just played back to
// back -- see bootChimeSteps below.
struct BootChimeStep {
  int note;
  int volume;
  int durationMs;
  unsigned long startAtMs;
};

// Hand-timed against the wave animation above: two slow opening notes
// during STILL, a faster run through BUILDING and into CHAOS, a tight
// run of short notes landing in CHAOS itself, one note easing into
// RESOLVING, then three held notes during SETTLED as a closing phrase
// once the wave has calmed down and the wordmark is about to appear.
const BootChimeStep bootChimeSteps[] = {
  {NOTE_E3, 8,  200, 0},
  {NOTE_E3, 9,  150, 280},

  {NOTE_G3, 9,  90,  460},
  {NOTE_A3, 9,  90,  570},
  {NOTE_B3, 10, 80,  680},
  {NOTE_D4, 10, 80,  780},
  {NOTE_E4, 10, 70,  880},
  {NOTE_G4, 10, 70,  970},

  {NOTE_A4, 10, 50,  1060},
  {NOTE_G4, 10, 50,  1130},
  {NOTE_B4, 10, 50,  1200},
  {NOTE_A4, 10, 50,  1270},
  {NOTE_C5, 10, 50,  1340},
  {NOTE_B4, 10, 50,  1410},

  {NOTE_G4, 10, 120, 1750},
  {NOTE_C5, 10, 120, 1900},
  {NOTE_E5, 10, 700, 2050}
};
const int numBootChimeSteps = sizeof(bootChimeSteps) / sizeof(bootChimeSteps[0]);

void setDisplayBrightness(uint8_t level) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(level);
}

// Draws one frame of the boot wave for a given point in the timeline
// (progress, 0.0 to 1.0). amplitude/frequency/jitter are derived from
// which phase progress falls in, each interpolated smoothly within
// its own phase (the "t" in each branch is 0.0 at that phase's start
// and 1.0 at its end) rather than jumping straight to the next phase's
// values. jitter adds a pseudo-random per-segment wobble (via a sine-
// based hash, not true randomness) on top of the smooth wave, used
// only during CHAOS/RESOLVING to make the wave look unstable before it
// settles. The wordmark's appearance is a hard cutoff at 50% computed
// opacity, not an actual fade -- it's either fully drawn or not drawn
// at all on any given frame.
void drawBootFrame(float progress) {
  display.clearDisplay();

  float amplitude = 0;
  float frequency = 1.0;
  float jitter = 0;
  float wordmarkOpacity = 0;

  if (progress < PHASE_STILL_END) {
    amplitude = 0;
    frequency = 1.0;
  } else if (progress < PHASE_BUILDING_END) {
    float t = (progress - PHASE_STILL_END) / (PHASE_BUILDING_END - PHASE_STILL_END);
    amplitude = t * 14;
    frequency = 1.0 + t * 2.5;
  } else if (progress < PHASE_CHAOS_END) {
    float t = (progress - PHASE_BUILDING_END) / (PHASE_CHAOS_END - PHASE_BUILDING_END);
    amplitude = 14 + t * 6;
    frequency = 3.5 + t * 1.5;
    jitter = t * 4;
  } else if (progress < PHASE_RESOLVING_END) {
    float t = (progress - PHASE_CHAOS_END) / (PHASE_RESOLVING_END - PHASE_CHAOS_END);
    amplitude = 20 - t * 8;
    frequency = 5.0 - t * 2.0;
    jitter = 4 * (1.0 - t);
  } else if (progress < PHASE_SETTLED_END) {
    amplitude = 12;
    frequency = 3.0;
    jitter = 0;
  } else if (progress < PHASE_WORDMARK_END) {
    amplitude = 12;
    frequency = 3.0;
    wordmarkOpacity = (progress - PHASE_SETTLED_END) / (PHASE_WORDMARK_END - PHASE_SETTLED_END);
  } else {
    amplitude = 12;
    frequency = 3.0;
    wordmarkOpacity = 1.0;
  }

  const int waveMidY = 36;
  const int waveLeft = 10;
  const int waveRight = SCREEN_WIDTH - 10;
  const int numSegments = 40;

  int prevX = waveLeft;
  int prevY = waveMidY;
  for (int i = 1; i <= numSegments; i++) {
    float xFrac = (float)i / numSegments;
    int x = waveLeft + (int)(xFrac * (waveRight - waveLeft));
    float phase = xFrac * frequency * 2 * PI;
    float y = waveMidY + amplitude * sin(phase + progress * 6.0);
    if (jitter > 0) {
      // Pseudo-random per-segment offset: hashes (segment index, boot
      // progress) through sin()/its fractional part rather than using
      // a true RNG, so the wobble is deterministic and repeatable.
      float noiseSeed = sin((i * 12.9898 + progress * 78.233)) * 43758.5453;
      float noise = (noiseSeed - floor(noiseSeed)) * 2.0 - 1.0;
      y += noise * jitter;
    }
    int yi = constrain((int)y, 0, SCREEN_HEIGHT - 1);
    display.drawLine(prevX, prevY, x, yi, SSD1306_WHITE);
    prevX = x;
    prevY = yi;
  }

  if (wordmarkOpacity >= 0.5) {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    const char* wordmark = "TELELELE";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(wordmark, 0, 0, &x1, &y1, &w, &h);
    int centeredX = (SCREEN_WIDTH - w) / 2;
    display.setCursor(centeredX, 50);
    display.print(wordmark);
  }

  display.display();
}

// Runs the boot animation, brightness ramp, and chime score all at
// once inside one loop, each on its own timer -- possible because every
// toneAC() call here passes true for its 4th argument (non-blocking),
// unlike the CHIME LIBRARY functions above, which all pass false and
// rely on delay() between notes. A blocking chime can't share time with
// a simultaneous animation; this one has to, so it isn't one.
void playBootSplash() {
  setDisplayBrightness(0);
  display.clearDisplay();
  display.display();
  delay(CRT_WARMUP_DELAY_MS); // the silent, blank pause before the wave starts

  Wire.setClock(400000); // boot only: faster I2C while drawing many frames in a row

  drawBootFrame(0.0);

  unsigned long startTime = millis();
  unsigned long lastFrameTime = 0;
  int chimeStepIndex = 0;
  bool chimeStepPlaying = false;
  unsigned long chimeStepEndsAt = 0;

  const uint8_t BRIGHTNESS_START = 26;
  const uint8_t BRIGHTNESS_RAMP_END = 178;
  const uint8_t BRIGHTNESS_FULL = 255;
  const unsigned long BRIGHTNESS_RAMP_DURATION_MS = 2500;

  while (millis() - startTime < BOOT_DURATION_MS) {
    unsigned long now = millis();
    unsigned long elapsed = now - startTime;

    // Brightness ramps from near-off up to BRIGHTNESS_RAMP_END over the
    // first 2.5s, then holds at full for whatever's left of the 3s
    // animation -- separate from, and slower than, the wave's own
    // phase timeline above.
    if (elapsed < BRIGHTNESS_RAMP_DURATION_MS) {
      float rampProgress = (float)elapsed / BRIGHTNESS_RAMP_DURATION_MS;
      uint8_t brightness = BRIGHTNESS_START
        + (uint8_t)(rampProgress * (BRIGHTNESS_RAMP_END - BRIGHTNESS_START));
      setDisplayBrightness(brightness);
    } else {
      setDisplayBrightness(BRIGHTNESS_FULL);
    }

    if (now - lastFrameTime >= BOOT_FRAME_INTERVAL_MS) {
      lastFrameTime = now;
      float progress = (float)elapsed / BOOT_DURATION_MS;
      drawBootFrame(progress);
    }

    // Fires the next chime note once its scheduled startAtMs has
    // passed and the previous note (if any) has finished -- notes
    // never overlap, but a note that's still playing when its slot
    // arrives just gets started right when the previous one ends,
    // rather than being skipped.
    if (!chimeStepPlaying && chimeStepIndex < numBootChimeSteps
        && elapsed >= bootChimeSteps[chimeStepIndex].startAtMs) {
      BootChimeStep step = bootChimeSteps[chimeStepIndex];
      toneAC(step.note, step.volume, step.durationMs, true);
      chimeStepPlaying = true;
      chimeStepEndsAt = now + step.durationMs;
      chimeStepIndex++;
    }
    if (chimeStepPlaying && now >= chimeStepEndsAt) {
      chimeStepPlaying = false;
    }
  }

  noToneAC();
  setDisplayBrightness(BRIGHTNESS_FULL);
  drawBootFrame(1.0); // guarantees the final frame (wordmark, full brightness) actually draws

  Wire.setClock(25000); // back to normal-operation I2C speed before loop() starts
}

void setup() {
  Serial.begin(115200);
  pinMode(7, OUTPUT);
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(BTN, INPUT_PULLUP);

  // Seed the encoder's initial combined state before attaching interrupts,
  // so the very first transition seen has a valid "old state" to compare
  // against rather than defaulting to 0 (which could register a false
  // tick if the encoder happens to power up sitting in state "11").
  encoderLastState = (digitalRead(CLK) << 1) | digitalRead(DT);

  attachInterrupt(digitalPinToInterrupt(CLK), handleEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT), handleEncoderChange, CHANGE);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed. Check wiring and I2C address.");
  } else {
    display.clearDisplay();
    display.display();
    playBootSplash();
  }
}

void loop() {
  unsigned long now = millis();

  // =====================================================================
  // SECTION 1: BUTTON HANDLING
  // =====================================================================
  // Recognizes: single/double click (clickCount, resolved once
  // clickWindow passes with no further click), several different
  // hold-and-release gestures (each with its own threshold constant and
  // a "*Fired" flag so it triggers exactly once per hold rather than
  // repeatedly while held), and a "spam-click" universal exit (6 clicks
  // in quick succession from any non-Tuning mode). Many of these
  // gestures only make sense in certain modes -- each block below
  // checks `mode` itself rather than assuming what's currently active.

  int btnState = digitalRead(BTN);
  bool justPressed  = (btnState == LOW  && lastBtnState == HIGH && (now - lastBtnTime > debounceDelay));
  bool justReleased = (btnState == HIGH && lastBtnState == LOW  && (now - lastBtnTime > debounceDelay));

  // inputLockedAfterLoad briefly ignores all button/encoder input right
  // after a successful EEPROM load, so the same button-release that
  // ended the hold-to-load gesture doesn't also immediately register as
  // a click in the mode it just switched into. Cleared on the next
  // release. Gates this section and Section 2 (encoder) only -- never
  // audio or display, per the standing rule on inputLockedAfterLoad.
  if (inputLockedAfterLoad) {
    if (justReleased) {
      inputLockedAfterLoad = false;
      lastBtnState = btnState;
      clickCount = 0;
      btnHoldStart = now;
      btnHoldFired = false;
    } else {
      lastBtnState = btnState;
    }

  } else {

  if (justPressed) {
    lastBtnTime = now;
    btnHoldStart = now;
    btnHoldFired = false;
    eepromPromptFired = false;
    loadHoldFired = false;
    lastButtonPressTime = now;

    // Spam-click exit: counts presses with short gaps between them
    // (SPAM_CLICK_MAX_GAP_MS), resetting the count whenever a gap is
    // too long. Hitting SPAM_CLICK_EXIT_COUNT presses in a row from any
    // non-Tuning mode force-exits back to Tuning immediately, as an
    // always-available "get me out of here" gesture independent of
    // whatever mode-specific click/hold logic is also in play.
    if (now - lastSpamClickTime <= SPAM_CLICK_MAX_GAP_MS) spamClickCount++;
    else spamClickCount = 1;
    lastSpamClickTime = now;

    if (spamClickCount >= SPAM_CLICK_EXIT_COUNT && mode != 0) {
      spamClickCount = 0;
      clickCount = 0;
      exitReplayMode();
    }

    clickCount++;
    if (clickCount == 1) firstClickTime = now;
  }

  if (justReleased) {
    unsigned long heldFor = now - btnHoldStart;

    // A release that's too long to be a click but too short to have
    // reached the hold-to-load threshold (Tuning mode only) is neither
    // -- discard the click so it doesn't get misread as one once
    // clickWindow passes.
    if (mode == 0 && heldFor >= LOAD_CLICK_MAX_MS && heldFor < LOAD_HOLD_THRESHOLD_MS) {
      clickCount = 0;
    }

    // Releasing quickly enough during the "Hold to create" prompt
    // (before SAVE_SCREEN_GRACE_PERIOD_MS) means the hold-to-create-Sound
    // gesture never actually reached its threshold -- drop the prompt
    // rather than leave it on screen for a press that's already over.
    if (saveScreenState == SAVE_SCREEN_MAKING_SOUND
        && (now - btnHoldStart) < SAVE_SCREEN_GRACE_PERIOD_MS) {
      saveScreenState = SAVE_SCREEN_NONE;
    }
  }

  if (btnState == LOW) {
    // Hold-to-load (Tuning mode only): reads whatever loop was last
    // saved to EEPROM and jumps straight into Playback Speed with it.
    if (mode == 0 && !loadHoldFired && (now - btnHoldStart >= LOAD_HOLD_THRESHOLD_MS)) {
      loadHoldFired = true;
      clickCount = 0;
      setSaveScreen(SAVE_SCREEN_LOADING);
      if (loadLoopFromEeprom()) {
        playEepromLoadedChime();
        playbackStartTime = now;
        if (!exactSpeedBaselineCaptured) {
          exactSpeedBaseline = exactSpeedPercent;
          exactSpeedBaselineCaptured = true;
        }
        mode = 2;
        setSaveScreen(SAVE_SCREEN_LOADED);
        Serial.println("Mode: Playback Speed (Loaded)");
      } else {
        playErrorTone();
        setSaveScreen(SAVE_SCREEN_SAVE_CANCELLED);
        Serial.println("Load failed: no saved loop in EEPROM.");
      }
      inputLockedAfterLoad = true;
      saveScreenState = SAVE_SCREEN_NONE;
    }

    // Hold-to-create-a-Sound, from any playback/effect mode: freezes
    // the current loop + pitch + fade for per-string triggering and
    // jumps to Sound mode (5).
    if ((mode == 2 || mode == 3 || mode == 4 || mode == 6 || mode == 7 || mode == 8) && !btnHoldFired && (now - btnHoldStart >= saveSoundHoldThreshold)) {
      btnHoldFired = true;
      clickCount = 0;
      noToneAC();
      saveCurrentAsSound();
      mode = 5;
      setSaveScreen(SAVE_SCREEN_SOUND_MADE);
      btnHoldStart = now;
      btnHoldFired = false;
      eepromPromptFired = false;
      Serial.println("Mode: Sound Tuning");
    }

    // While that hold is still building up (past the grace period, not
    // yet at threshold), show the "Hold to create" prompt so the player
    // gets feedback that something is happening before the gesture
    // actually fires.
    if ((mode == 2 || mode == 3 || mode == 4 || mode == 6 || mode == 7 || mode == 8)
        && !btnHoldFired && saveScreenState != SAVE_SCREEN_SOUND_MADE
        && (now - btnHoldStart) >= SAVE_SCREEN_GRACE_PERIOD_MS) {
      if (saveScreenState != SAVE_SCREEN_MAKING_SOUND) {
        setSaveScreen(SAVE_SCREEN_MAKING_SOUND);
      }
    }

    // Hold-to-save, Sound mode only: a two-stage confirm gesture below
    // (release-then-rehold) protects against accidentally overwriting
    // the one EEPROM save slot with a single careless hold.
    if (mode == 5 && !eepromPromptFired && (now - btnHoldStart >= eepromPromptHoldThreshold)) {
      eepromPromptFired = true;
      clickCount = 0;
      noToneAC();
      playEepromPromptChime();
      awaitingEepromConfirm = true;
      eepromPromptTime = now;
      eepromReHolding = false;
      setSaveScreen(SAVE_SCREEN_PROMPT);
    }

    // After a brief pause showing "Save sound?", switch the prompt text
    // to tell the player what to actually do next.
    if (awaitingEepromConfirm && saveScreenState == SAVE_SCREEN_PROMPT
        && (now - saveScreenShownAt >= 900)) {
      setSaveScreen(SAVE_SCREEN_RELEASE_TO_CONFIRM);
    }

    // The player has released and is holding again -- this is the
    // confirming re-hold. eepromReHoldStart anchors its own threshold
    // (eepromConfirmHoldThreshold) below, separate from the original
    // hold that triggered the prompt.
    if (awaitingEepromConfirm && !eepromReHolding) {
      eepromReHolding = true;
      eepromReHoldStart = now;
      if (saveScreenState == SAVE_SCREEN_RELEASE_TO_CONFIRM) {
        setSaveScreen(SAVE_SCREEN_HOLD_TO_CONFIRM);
      }
    }
    if (awaitingEepromConfirm && eepromReHolding && (now - eepromReHoldStart >= eepromConfirmHoldThreshold)) {
      saveLoopToEeprom();
      playEepromConfirmedChime();
      awaitingEepromConfirm = false;
      eepromReHolding = false;
      setSaveScreen(SAVE_SCREEN_SOUND_SAVED);
    }
  } else {
    // Button is up. Releasing mid-rehold cancels just the rehold
    // attempt (awaitingEepromConfirm itself stays true -- the player
    // gets another chance until eepromAbortTimeout, handled below).
    if (awaitingEepromConfirm) eepromReHolding = false;

    if (saveScreenState == SAVE_SCREEN_MAKING_SOUND) {
      saveScreenState = SAVE_SCREEN_NONE;
    }
  }

  // The whole release-then-rehold confirm gesture times out and cancels
  // if it isn't completed within eepromAbortTimeout of the original
  // prompt firing -- prevents an abandoned save attempt from silently
  // sitting in a half-confirmed state forever.
  if (awaitingEepromConfirm && (now - eepromPromptTime >= eepromAbortTimeout)) {
    awaitingEepromConfirm = false;
    eepromReHolding = false;
    playEepromAbortedChime();
    setSaveScreen(SAVE_SCREEN_SAVE_CANCELLED);
    Serial.println("EEPROM save aborted (timeout).");
  }

  // Brief, self-clearing confirmation screens (sound made/saved,
  // cancelled, loaded) all expire after the same fixed duration rather
  // than needing their own individual timers.
  if ((saveScreenState == SAVE_SCREEN_SOUND_MADE
       || saveScreenState == SAVE_SCREEN_SOUND_SAVED
       || saveScreenState == SAVE_SCREEN_SAVE_CANCELLED
       || saveScreenState == SAVE_SCREEN_LOADED)
      && (now - saveScreenShownAt >= BRIEF_SAVE_SCREEN_DURATION_MS)) {
    saveScreenState = SAVE_SCREEN_NONE;
  }

  lastBtnState = btnState;

  // Mode 0 (Tuning) still has the hold-to-load gesture. Mode 1 (Record)
  // no longer has any competing hold gesture -- the old "hold to escalate
  // into exact record" step is gone now that Record IS the one engine.
  // Modes 2/3/4/6/7/8 still have the hold-to-create-a-Sound gesture, and
  // mode 5 (Sound) still has the hold-to-save-to-EEPROM gesture.
  bool modeHasHoldGesture = (mode == 0 || mode == 2 || mode == 3 || mode == 4
                              || mode == 5 || mode == 6 || mode == 7 || mode == 8);

  // Click resolution: waits until clickWindow has passed since the
  // first click with no further click arriving, so a double-click has
  // time to actually complete before being treated as a single. Also
  // waits for the button to be fully released if the current mode has
  // a competing hold gesture, so a long hold that happens to start as
  // what looked like one click doesn't simultaneously fire a click
  // action underneath it.
  if (clickCount > 0 && (now - firstClickTime > clickWindow) && !awaitingEepromConfirm
      && (!modeHasHoldGesture || btnState == HIGH)) {
    if (clickCount == 2) {
      // Double-click: steps forward through the effect chain
      // Speed -> Pitch -> Fade -> Tremolo -> Vibrato -> Bitcrusher,
      // then wraps back around to Speed.
      if (mode == 2) {
        mode = 3; playPitchModeChime();
        Serial.println("Mode: Playback Pitch");
      } else if (mode == 3) {
        mode = 4; playFadeModeChime(); fadeAmountPercent = 0;
        Serial.println("Mode: Playback Fade");
      } else if (mode == 4) {
        mode = 6; playFadeModeChime();
        Serial.println("Mode: Tremolo");
      } else if (mode == 6) {
        mode = 7; playFadeModeChime();
        Serial.println("Mode: Vibrato");
      } else if (mode == 7) {
        mode = 8; playFadeModeChime();
        Serial.println("Mode: Bitcrusher");
      } else if (mode == 8) {
        if (!exactSpeedBaselineCaptured) {
          exactSpeedBaseline = exactSpeedPercent;
          exactSpeedBaselineCaptured = true;
        }
        mode = 2; playbackStartTime = now; playSpeedModeChime();
        Serial.println("Mode: Playback Speed");
      }
    } else if (clickCount == 1) {
      // Single click in Tuning, after the post-exit cooldown has
      // passed: starts a fresh recording. Everything reset here is
      // state from any previous recording/playback that shouldn't
      // carry over into this new one.
      if (mode == 0 && (now - exitedAtTime >= postExitCooldownMs)) {
        exactEventCount = 0;
        exactRecordOverflowed = false;
        exactCapacityWarningGiven = false;
        capacityWarningFlashStartTime = -1;
        for (int gi = 0; gi < 4; gi++) {
          isGliding[gi] = false;
          glideAnchorFreq[gi] = 0;
          touchDownTime[gi] = 0;
        }
        lastActiveString = -1;
        pitchShiftPercent = 100;
        exactSpeedPercent = 100;
        exactLoopDuration = 0;
        exactLoopStartIndex = 0;
        fadeAmountPercent = 0;
        savedFadeAmountPercent = 0;
        exactRecordStartTime = now;
        mode = 1;
        playExactRecordChime();
        Serial.print("Mode: Record | Tuning preset: ");
        Serial.println(tuningPreset);
      } else if (mode == 1) {
        if (exactEventCount == 0) {
          playErrorTone();
          Serial.println("No samples recorded. Staying in Record.");
        } else {
          computeExactLoopBoundaries();
          playPlaybackEntryChime();
          playbackStartTime = now;
          if (!exactSpeedBaselineCaptured) {
            exactSpeedBaseline = exactSpeedPercent;
            exactSpeedBaselineCaptured = true;
          }
          mode = 2;
          Serial.println("Mode: Playback Speed");
        }
      } else if (mode == 2) {
        if (exactSpeedPercent != exactSpeedBaseline) {
          exactSpeedRememberedValue = exactSpeedPercent;
          exactSpeedPercent = exactSpeedBaseline;
          playbackStartTime = now;
        } else if (exactSpeedRememberedValue != -1) {
          exactSpeedPercent = exactSpeedRememberedValue;
          playbackStartTime = now;
        }
      } else if (mode == 3) {
        if (pitchShiftPercent != PITCH_BASELINE) {
          pitchRememberedValue = pitchShiftPercent;
          pitchShiftPercent = PITCH_BASELINE;
        } else if (pitchRememberedValue != -1) {
          pitchShiftPercent = pitchRememberedValue;
        }
      } else if (mode == 4) {
        if (fadeAmountPercent != FADE_BASELINE) {
          fadeRememberedValue = fadeAmountPercent;
          fadeAmountPercent = FADE_BASELINE;
        } else if (fadeRememberedValue != -1) {
          fadeAmountPercent = fadeRememberedValue;
        }
      } else if (mode == 6) {
        tremoloOn = !tremoloOn;
        Serial.print("Tremolo: "); Serial.println(tremoloOn ? "ON" : "OFF");
      } else if (mode == 7) {
        vibratoOn = !vibratoOn;
        Serial.print("Vibrato: "); Serial.println(vibratoOn ? "ON" : "OFF");
      } else if (mode == 8) {
        bitcrushOn = !bitcrushOn;
        Serial.print("Bitcrusher: "); Serial.println(bitcrushOn ? "ON" : "OFF");
      }
    }
    clickCount = 0;
  }

  } // end of (!inputLockedAfterLoad) block

  // =====================================================================
  // SECTION 2: ENCODER HANDLING
  // =====================================================================
  // encoderDelta is written from handleEncoderChange() (an interrupt
  // handler -- see near the top of the file), so it's read and cleared
  // here inside a noInterrupts()/interrupts() pair to avoid a race
  // where an interrupt fires mid-read. Each whole tick recorded since
  // the last loop() iteration is replayed one at a time through
  // processSmoothedTick(), since more than one tick can accumulate
  // between iterations if the encoder is turned quickly.

  if (!inputLockedAfterLoad) {
    noInterrupts();
    int delta = encoderDelta;
    encoderDelta = 0;
    interrupts();

    if (delta != 0) {
      int direction = (delta > 0) ? 1 : -1;
      int ticks = abs(delta);
      for (int t = 0; t < ticks; t++) {
        processSmoothedTick(direction);
      }
    }
    if (knobActive && (now - lastTurnTime > idleTimeout)) knobActive = false;
  } else {
    // Still drain and discard encoderDelta even while input is locked,
    // so ticks that happened during the lock don't all suddenly apply
    // at once the moment it clears.
    noInterrupts();
    encoderDelta = 0;
    interrupts();
  }

  // =====================================================================
  // SECTION 3: MODE AUDIO BEHAVIOURS
  // =====================================================================
  // Reads the four strings and drives toneAC accordingly, specific to
  // whatever mode is currently active. Tuning (0), Record (1), and
  // Sound (5) all read the pots directly every iteration; the playback
  // modes (2/3/4/6/7/8) instead read from the already-recorded loop via
  // runExactPlayback(), since there's nothing live to read from the
  // strings in those modes -- only mode 3 (Pitch) also reads pots, to
  // let touching a string bend playback pitch in real time.

  if (mode == 0) {
    // Tuning: every touched string sounds at once (toneAC itself is
    // still monophonic, so in practice the last one written each frame
    // wins) -- there's no recording or other state to manage here, just
    // direct pot-to-pitch mapping per the active tuningPreset.
    int pot[4] = { analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3) };
    bool isPlaying = false;
    for (int i = 0; i < 4; i++) {
      if (pot[i] > threshold) {
        int freq = map(pot[i], 1023, 30, tunings[tuningPreset][i][0], tunings[tuningPreset][i][1]);
        toneAC(freq, 10, 0, true);
        isPlaying = true;
      }
    }
    if (!isPlaying) noToneAC();

  } else if (mode == 1) {
    int pot[4] = { analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3) };
    int newActiveString = lastActiveString;
    for (int i = 0; i < 4; i++) {
      if (pot[i] > threshold && prevPot[i] <= threshold) newActiveString = i;
    }
    bool anyActive = false;
    for (int i = 0; i < 4; i++) {
      if (pot[i] > threshold) anyActive = true;
    }
    unsigned long elapsed = now - exactRecordStartTime;

    if (anyActive && newActiveString >= 0) {
      bool isNewTouch = (newActiveString != lastActiveString);
      lastActiveString = newActiveString;
      int freq = map(pot[newActiveString], 1023, 30,
                     tunings[tuningPreset][newActiveString][0],
                     tunings[tuningPreset][newActiveString][1]);
      toneAC(freq, 10, 0, true);

      if (isNewTouch) {
        // Touch-down: start the settle window and commit the onset
        // event immediately. The settle window lets the anchor re-pin
        // freely for a brief period, absorbing the mechanical settling
        // of a finger landing on the string, before glide-detection
        // (drift-from-anchor) starts being trusted.
        glideAnchorFreq[newActiveString] = freq;
        isGliding[newActiveString] = false;
        touchDownTime[newActiveString] = now;
        lastGlideSampleTime = now;
        if (!commitExactEvent((uint16_t)freq, elapsed)) {
          // Buffer was already full at the moment of touch-down. Not a
          // glide-tail loss, but still "no room" -- per the updated
          // behavior, any way of hitting the cap ends the recording and
          // moves to playback rather than leaving the user stuck.
          exactRecordOverflowed = true;
        }
      } else if (now - touchDownTime[newActiveString] < GLIDE_TOUCHDOWN_SETTLE_MS) {
        // Still settling: let the anchor track freely, no comparison.
        glideAnchorFreq[newActiveString] = freq;
      } else if (!isGliding[newActiveString]) {
        // Settled. Check for drift past the jitter threshold -- a
        // percentage of the anchor pitch, not a flat Hz value, so
        // sensitivity feels the same on low and high strings alike.
        int drift = freq - glideAnchorFreq[newActiveString];
        if (abs(drift) > glideJitterThresholdForFreq(glideAnchorFreq[newActiveString])) {
          // Glide onset: commit the anchor (where it was steady until
          // just now), then switch into full-rate sampling for the
          // remainder of this press, regardless of whether movement
          // continues -- only release ends glide mode, per design.
          if (!commitExactEvent((uint16_t)glideAnchorFreq[newActiveString], elapsed)) {
            exactRecordOverflowed = true;
          } else {
            isGliding[newActiveString] = true;
            lastGlideSampleTime = now;
          }
        }
        // Within threshold: steady hold, nothing to record.
      } else {
        // Already gliding: sample at the glide rate until release.
        if (now - lastGlideSampleTime >= GLIDE_SAMPLE_INTERVAL_MS) {
          if (!commitExactEvent((uint16_t)freq, elapsed)) {
            exactRecordOverflowed = true;
          } else {
            lastGlideSampleTime = now;
          }
        }
      }

    } else {
      // No string touched: if a string was just released, write one
      // release marker at the precise release time (not snapped to any
      // fixed interval), then reset that string's glide state.
      if (lastActiveString >= 0 && !commitExactEvent(0, elapsed)) {
        exactRecordOverflowed = true;
      }
      if (lastActiveString >= 0) {
        isGliding[lastActiveString] = false;
        glideAnchorFreq[lastActiveString] = 0;
        touchDownTime[lastActiveString] = 0;
      }
      lastActiveString = -1;
      noToneAC();
    }

    if (exactRecordOverflowed) {
      // Event buffer is full, however it happened: mid-glide, at a
      // fresh touch-down, or at a release marker. Whichever path got
      // here, the recording ends now and moves straight to playback
      // rather than leaving the user stuck with a beep and nowhere to
      // go. Checked once per loop iteration, outside both branches
      // above, so it fires regardless of which one set the flag.
      noToneAC();
      playExactRecordOverflowChime();
      Serial.println("Recording cut short: event buffer full.");
      computeExactLoopBoundaries();
      playbackStartTime = now;
      if (!exactSpeedBaselineCaptured) {
        exactSpeedBaseline = exactSpeedPercent;
        exactSpeedBaselineCaptured = true;
      }
      mode = 2;
      Serial.println("Mode: Playback Speed");
    }
    for (int i = 0; i < 4; i++) prevPot[i] = pot[i];

  } else if (mode == 2 || mode == 4 || mode == 6 || mode == 7 || mode == 8) {
    runExactPlayback(now);

  } else if (mode == 3) {
    int pot[4] = { analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3) };
    int highestPot = 0; bool anyActive = false;
    for (int i = 0; i < 4; i++) {
      if (pot[i] > threshold) { anyActive = true; if (pot[i] > highestPot) highestPot = pot[i]; }
    }
    if (anyActive) pitchShiftPercent = constrain(map(highestPot, threshold, 1023, 25, 400), 25, 400);
    runExactPlayback(now);

  } else if (mode == 5) {
    // Sound mode: each string independently tracks its own trigger
    // state (soundTriggerActive/Released/etc, all per-string arrays),
    // so up to 4 "voices" can be logically active/releasing at once
    // even though only one can actually be heard (toneAC is
    // monophonic) -- the priority loop below decides which.
    int pot[4] = { analogRead(A0), analogRead(A1), analogRead(A2), analogRead(A3) };
    activeStringForDisplay = -1;
    for (int i = 0; i < 4; i++) {
      bool touchedNow = (pot[i] > threshold);
      bool wasTouched  = (prevPot[i] > threshold);
      if (touchedNow) {
        // percent here is the per-string pitch bend used by this
        // Sound's playback (soundTriggerPitchPercent), derived from
        // where on the pot the string is currently pressed -- not
        // related to the global Playback Pitch mode at all.
        int rawFreq = map(pot[i], 1023, 30, tunings[tuningPreset][i][0], tunings[tuningPreset][i][1]);
        int baseFreq = tunings[tuningPreset][i][0];
        int percent = map(rawFreq, baseFreq, tunings[tuningPreset][i][1], 100, 200);
        percent = constrain(percent, 25, 400);
        if (!wasTouched) {
          // Fresh touch: (re)trigger this string's voice from its own
          // start, independent of whatever any other string is doing.
          soundTriggerPitchPercent[i] = percent;
          soundTriggerActive[i] = true;
          soundTriggerReleased[i] = false;
          soundTriggerStart[i] = now;
        } else if (soundTriggerActive[i]) {
          soundTriggerPitchPercent[i] = percent;
        }
      } else {
        if (wasTouched && soundTriggerActive[i] && !soundTriggerReleased[i]) {
          soundTriggerReleased[i] = true;
          soundTriggerReleaseTime[i] = now;
        }
      }
      prevPot[i] = pot[i];
    }
    // Highest string index wins when more than one is active at once
    // -- a simple, fixed tie-break (not connected to anything visual;
    // stringDisplayBoxOrder above is a separate, unrelated layout
    // array). Only the winning string's voice actually reaches
    // runSoundStringExact()/toneAC this frame.
    bool anyPlayed = false;
    for (int i = 3; i >= 0; i--) {
      if (soundTriggerActive[i]) {
        runSoundStringExact(i, now);
        anyPlayed = true;
        activeStringForDisplay = i;
        break;
      }
    }
    if (!anyPlayed && !awaitingEepromConfirm) noToneAC();
  }

  // =====================================================================
  // SECTION 4: DISPLAY
  // =====================================================================

  if (now - lastDisplayFrameTime >= DISPLAY_FRAME_INTERVAL_MS) {
    lastDisplayFrameTime = now;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    bool recentEncoderChange = (now - lastEncoderChangeTime < ENCODER_FLASH_DURATION_MS);
    bool iconFrameB = ((now / 400) % 2 == 0);

    if (saveScreenState != SAVE_SCREEN_NONE && !inputLockedAfterLoad) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("SAVE");

      switch (saveScreenState) {
        case SAVE_SCREEN_MAKING_SOUND:
          drawSoundCreateIcon(SCREEN_WIDTH / 2 - 6, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Hold to create", nullptr);
          break;
        case SAVE_SCREEN_SOUND_MADE:
          drawSoundCreateIcon(SCREEN_WIDTH / 2 - 6, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Sound created", nullptr);
          break;
        case SAVE_SCREEN_PROMPT:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Save sound?", nullptr);
          break;
        case SAVE_SCREEN_RELEASE_TO_CONFIRM:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Release, then hold", "to save");
          break;
        case SAVE_SCREEN_HOLD_TO_CONFIRM:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Hold to confirm", nullptr);
          break;
        case SAVE_SCREEN_SOUND_SAVED:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Sound saved", nullptr);
          break;
        case SAVE_SCREEN_SAVE_CANCELLED:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Save cancelled", nullptr);
          break;
        case SAVE_SCREEN_LOADING:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Loading...", nullptr);
          break;
        case SAVE_SCREEN_LOADED:
          drawFolderIcon(SCREEN_WIDTH / 2 - 7, BLUE_TOP + 2, iconFrameB);
          drawSaveScreenMessage("Loaded", nullptr);
          break;
        default:
          break;
      }

    } else if (mode == 0) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("TUNING");
      drawHomeIcon(SCREEN_WIDTH - 18, 2);

      display.setTextSize(2);
      int16_t x1, y1; uint16_t w, h;
      display.getTextBounds(tuningNames[tuningPreset], 0, 0, &x1, &y1, &w, &h);
      display.setCursor((SCREEN_WIDTH - w) / 2, BLUE_TOP + 16);
      display.print(tuningNames[tuningPreset]);

      if (recentEncoderChange) {
        display.fillTriangle(SCREEN_WIDTH / 2 - 6, BLUE_TOP + 4,
                              SCREEN_WIDTH / 2 + 6, BLUE_TOP + 4,
                              SCREEN_WIDTH / 2, BLUE_TOP - 2, SSD1306_WHITE);
      }

    } else if (mode == 1) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("REC");
      drawRecordIcon(SCREEN_WIDTH - 11, 8, iconFrameB);

      // Tap/Hold/Glide indicator: reflects what's actually happening to
      // the currently active recording voice right now. Rather than
      // showing both possibilities during the ambiguous opening window
      // (the engine genuinely can't yet tell a tap from a hold), this
      // defaults to "Tap" -- taps are the more common gesture in
      // practice -- and only switches to "Hold" once REC_STATE_DISPLAY_
      // DWELL_MS has elapsed without a release. This is display-only:
      // it does not touch GLIDE_TOUCHDOWN_SETTLE_MS or any other real
      // recording-engine timing. Glide is checked first and overrides
      // both immediately and unconditionally once detected. All three
      // states are mutually exclusive now, so they share one fixed x,
      // centered for the longest word ("Glide") so there's no shift
      // switching between states.
      const unsigned long REC_STATE_DISPLAY_DWELL_MS = 350;
      const int REC_STATE_X = 51;
      const int REC_STATE_Y = 4;

      if (lastActiveString >= 0) {
        display.setCursor(REC_STATE_X, REC_STATE_Y);
        if (isGliding[lastActiveString]) {
          display.print("Glide");
        } else if (now - touchDownTime[lastActiveString] < REC_STATE_DISPLAY_DWELL_MS) {
          display.print("Tap");
        } else {
          display.print("Hold");
        }
      }

      // Exact events-recorded readout, second-smallest text size, placed
      // at the top-left of the blue content area -- the same corner
      // every other mode reserves for its headline readout.
      display.setTextSize(1);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(exactEventCount);
      display.print("/");
      display.print(MAX_EXACT_EVENTS);

      display.setTextSize(2);
      unsigned long elapsed = now - exactRecordStartTime;
      display.setCursor(10, BLUE_TOP + 16);
      display.print(elapsed / 1000.0, 1);
      display.print("s");

      // Capacity bar: full screen width, no side margins (so the leading
      // edge is unambiguous right up to the screen edge), flush to the
      // bottom edge, solid fill -- no border-then-fill, per design.
      const int REC_BAR_THICKNESS = 8;
      const int REC_BAR_TOP = SCREEN_HEIGHT - REC_BAR_THICKNESS;
      const int REC_BAR_W = SCREEN_WIDTH;
      const int REC_NOTCH_W = 4;
      const int REC_MARKER_H = 4;
      const int REC_MARKER_GAP = 1;
      const int REC_MARKER_Y = REC_BAR_TOP - REC_MARKER_GAP - REC_MARKER_H;

      int recMarkerX = (int)((long)REC_BAR_W * MAX_EXACT_EVENTS_WARN_THRESHOLD / MAX_EXACT_EVENTS) - (REC_NOTCH_W / 2);
      int recFillW = (int)((long)exactEventCount * REC_BAR_W / MAX_EXACT_EVENTS);

      if (recFillW > 0) {
        if (recFillW <= recMarkerX) {
          display.fillRect(0, REC_BAR_TOP, recFillW, REC_BAR_THICKNESS, SSD1306_WHITE);
        } else {
          display.fillRect(0, REC_BAR_TOP, recMarkerX, REC_BAR_THICKNESS, SSD1306_WHITE);
          int recSecondSegmentW = recFillW - (recMarkerX + REC_NOTCH_W);
          if (recSecondSegmentW > 0) {
            display.fillRect(recMarkerX + REC_NOTCH_W, REC_BAR_TOP,
                              recSecondSegmentW, REC_BAR_THICKNESS, SSD1306_WHITE);
          }
        }
      }

      // Permanent marker line above the bar at the 80%-warning x-position
      // (lines up with the notch cut into the fill once it's reached).
      // Flashes for CAPACITY_WARNING_FLASH_DURATION_MS the instant the
      // warning fires, outlasting the brief chime by design so the cue
      // is visible well after the sound has finished.
      bool recShowMarker = true;
      if (capacityWarningFlashStartTime >= 0) {
        unsigned long sinceFlash = now - (unsigned long)capacityWarningFlashStartTime;
        if (sinceFlash < CAPACITY_WARNING_FLASH_DURATION_MS) {
          recShowMarker = ((sinceFlash / CAPACITY_WARNING_FLASH_HALF_PERIOD_MS) % 2 == 0);
        }
      }
      if (recShowMarker) {
        display.fillRect(recMarkerX, REC_MARKER_Y, REC_NOTCH_W, REC_MARKER_H, SSD1306_WHITE);
      }

    } else if (mode == 3) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("PITCH");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(pitchShiftPercent != PITCH_BASELINE);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(pitchShiftPercent);
      display.print("%");

      const int PITCH_STRING_LEFT_X = 8;
      const int PITCH_STRING_RIGHT_X = 120;
      const int PITCH_STRING_Y = BLUE_TOP + 32;
      const int PITCH_MAX_DOWN_BEND = 8;
      const int PITCH_MAX_UP_BEND = 16;
      const int PITCH_STRING_SEGMENTS = 8;
      const unsigned long PITCH_PULSE_PERIOD_MS = 1400;
      const float PITCH_PULSE_MIN_R = 1.5;
      const float PITCH_PULSE_MAX_R = 3.0;

      float pitchBendFrac;
      int pitchMaxBend;
      if (pitchShiftPercent < PITCH_BASELINE) {
        pitchBendFrac = (float)(PITCH_BASELINE - pitchShiftPercent) / 75.0;
        pitchMaxBend = PITCH_MAX_DOWN_BEND;
      } else if (pitchShiftPercent > PITCH_BASELINE) {
        pitchBendFrac = (float)(pitchShiftPercent - PITCH_BASELINE) / 300.0;
        pitchMaxBend = -PITCH_MAX_UP_BEND;
      } else {
        pitchBendFrac = 0;
        pitchMaxBend = 0;
      }
      float pitchBendAmount = pitchBendFrac * pitchMaxBend;

      int pitchPrevX = PITCH_STRING_LEFT_X;
      int pitchPrevY = PITCH_STRING_Y;
      int pitchPeakX = (PITCH_STRING_LEFT_X + PITCH_STRING_RIGHT_X) / 2;
      int pitchPeakY = PITCH_STRING_Y;
      for (int pi = 1; pi <= PITCH_STRING_SEGMENTS; pi++) {
        float pitchT = (float)pi / PITCH_STRING_SEGMENTS;
        float pitchBowFrac = sin(pitchT * PI);
        int pitchY = PITCH_STRING_Y + (int)(pitchBendAmount * pitchBowFrac);
        int pitchX = PITCH_STRING_LEFT_X + (int)(pitchT * (PITCH_STRING_RIGHT_X - PITCH_STRING_LEFT_X));
        display.drawLine(pitchPrevX, pitchPrevY, pitchX, pitchY, SSD1306_WHITE);
        if (pi == PITCH_STRING_SEGMENTS / 2) {
          pitchPeakX = pitchX;
          pitchPeakY = pitchY;
        }
        pitchPrevX = pitchX;
        pitchPrevY = pitchY;
      }

      unsigned long pitchPulsePhaseMs = now % PITCH_PULSE_PERIOD_MS;
      float pitchPulseWave = (sin((float)pitchPulsePhaseMs / PITCH_PULSE_PERIOD_MS * 2 * PI) + 1.0) / 2.0;
      float pitchPulseR = PITCH_PULSE_MIN_R + pitchPulseWave * (PITCH_PULSE_MAX_R - PITCH_PULSE_MIN_R);
      display.fillCircle(pitchPeakX, pitchPeakY, (int)round(pitchPulseR), SSD1306_WHITE);

    } else if (mode == 2) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("SPEED");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(exactSpeedPercent != exactSpeedBaseline);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(exactSpeedPercent);
      display.print("%");

      const int RUN_LEFT_X = 6;
      const int RUN_RIGHT_X = 122;
      const int RUN_TRACK_Y = 58;
      const int RUN_BLOCK_SIZE = 6;
      const int RUN_BOUNCE_HEIGHT = 6;
      const float RUN_SPEED_BASELINE_HZ = 2.0;

      if (exactLoopDuration > 0) {
        unsigned long scaledLoopLen = (unsigned long)((unsigned long long)exactLoopDuration * 100 / exactSpeedPercent);
        unsigned long playPos = (now - playbackStartTime) % scaledLoopLen;
        int runX = RUN_LEFT_X + (int)((long)playPos * (RUN_RIGHT_X - RUN_LEFT_X) / scaledLoopLen);

        float bounceHz = RUN_SPEED_BASELINE_HZ * ((float)exactSpeedPercent / 100.0);
        float bouncePhase = (now / 1000.0) * bounceHz * 2 * PI;
        float bounceFrac = abs(sin(bouncePhase));
        int yOffset = (int)(bounceFrac * RUN_BOUNCE_HEIGHT);

        display.fillRect(runX - RUN_BLOCK_SIZE / 2, RUN_TRACK_Y - RUN_BLOCK_SIZE - yOffset, RUN_BLOCK_SIZE, RUN_BLOCK_SIZE, SSD1306_WHITE);
      }
      display.drawFastHLine(RUN_LEFT_X, RUN_TRACK_Y, RUN_RIGHT_X - RUN_LEFT_X, SSD1306_WHITE);

    } else if (mode == 4) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("FADE");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(fadeAmountPercent != FADE_BASELINE);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(fadeAmountPercent);
      display.print("%");

      const int fadeBarLeft = 4;
      const int fadeBarRight = SCREEN_WIDTH - 4;
      const int fadeBarBottomY = BLUE_TOP + 42;
      const int fadeBarFullTopY = BLUE_TOP + 30;
      const int fadeBarMinTopY = fadeBarBottomY;
      const int fadeDotY = fadeBarFullTopY - 6;

      int fadeRightEdgeY = fadeBarFullTopY + (int)((long)fadeAmountPercent * (fadeBarMinTopY - fadeBarFullTopY) / 100);

      display.fillTriangle(fadeBarLeft, fadeBarFullTopY, fadeBarRight, fadeRightEdgeY, fadeBarLeft, fadeBarBottomY, SSD1306_WHITE);
      display.fillTriangle(fadeBarRight, fadeRightEdgeY, fadeBarRight, fadeBarBottomY, fadeBarLeft, fadeBarBottomY, SSD1306_WHITE);

      if (exactLoopDuration > 0) {
        unsigned long scaledLoopLen = (unsigned long)((unsigned long long)exactLoopDuration * 100 / exactSpeedPercent);
        unsigned long playPos = (now - playbackStartTime) % scaledLoopLen;
        int fadeDotX = map(playPos, 0, scaledLoopLen, fadeBarLeft, fadeBarRight);
        display.fillCircle(fadeDotX, fadeDotY, 2, SSD1306_WHITE);
      }

    } else if (mode == 5) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("SOUND");
      drawPluckIcon(SCREEN_WIDTH - 18, 2);

      if (activeStringForDisplay >= 0) {
        const int vibCenterX = SCREEN_WIDTH / 2;
        const int vibCenterY = BLUE_TOP + 16;
        const int vibWidth = 36;
        const float vibAmplitude = 4.0;
        const float vibSpeed = 0.03;

        int prevX = vibCenterX - vibWidth / 2;
        int prevY = vibCenterY;
        const int vibSegments = 8;
        for (int i = 1; i <= vibSegments; i++) {
          float xFrac = (float)i / vibSegments;
          int x = (vibCenterX - vibWidth / 2) + (int)(xFrac * vibWidth);
          float y = vibCenterY + vibAmplitude * sin(xFrac * PI * 3 + now * vibSpeed);
          int yi = (int)y;
          display.drawLine(prevX, prevY, x, yi, SSD1306_WHITE);
          prevX = x;
          prevY = yi;
        }
      }

      const int boxWidth = 26;
      const int boxGap = 6;
      const int totalBoxesWidth = (boxWidth * 4) + (boxGap * 3);
      const int boxesStartX = (SCREEN_WIDTH - totalBoxesWidth) / 2;

      for (int i = 0; i < 4; i++) {
        int displaySlot = stringDisplayBoxOrder[i];
        int boxX = boxesStartX + displaySlot * (boxWidth + boxGap);
        if (i == activeStringForDisplay) {
          display.fillRect(boxX, BLUE_TOP + 40, boxWidth, 12, SSD1306_WHITE);
        } else {
          display.drawRect(boxX, BLUE_TOP + 40, boxWidth, 12, SSD1306_WHITE);
        }
      }

    } else if (mode == 6) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("TREMOLO");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(tremoloOn);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(tremoloKnob);
      display.print("%");

      float tremRate = tremoloRateFromKnob();
      float tremDepthVal = tremoloDepthFromKnob();
      float tremPhase = (now / 1000.0) * tremRate * 2 * PI;
      float tremOsc = (sin(tremPhase) + 1.0) / 2.0;
      float tremMinVolFactor = 1.0 - (tremDepthVal / 100.0);
      float tremVolFactor = tremMinVolFactor + (1.0 - tremMinVolFactor) * tremOsc;

      const int tremBarMaxH = 11;
      const int tremBarW = 60;
      const int tremBarX = 34;
      const int tremBarBottomY = BLUE_TOP + 42;
      int tremCurH = (int)(tremVolFactor * tremBarMaxH);
      display.fillRect(tremBarX, tremBarBottomY - tremCurH, tremBarW, tremCurH, SSD1306_WHITE);

    } else if (mode == 7) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("VIBRATO");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(vibratoOn);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(vibratoKnob);
      display.print("%");

      float vibRate = vibratoRateFromKnob();
      float vibDepthVal = vibratoDepthFromKnob();
      float vibPhase = (now / 1000.0) * vibRate * 2 * PI;

      const int vibWaveSegments = 20;
      const int vibWaveLeft = 14;
      const int vibWaveRight = 114;
      const int vibWaveY = 42;
      float vibWaveAmp = vibDepthVal * 1.2;

      int vibPrevX = vibWaveLeft;
      int vibPrevY = vibWaveY + (int)(vibWaveAmp * sin(vibPhase));
      for (int vi = 1; vi <= vibWaveSegments; vi++) {
        float vibXFrac = (float)vi / vibWaveSegments;
        int vibX = vibWaveLeft + (int)(vibXFrac * (vibWaveRight - vibWaveLeft));
        float vibY = vibWaveY + vibWaveAmp * sin(vibPhase + vibXFrac * 4 * PI);
        int vibYi = (int)vibY;
        display.drawLine(vibPrevX, vibPrevY, vibX, vibYi, SSD1306_WHITE);
        vibPrevX = vibX;
        vibPrevY = vibYi;
      }

      int vibDotY = vibWaveY + (int)(vibWaveAmp * sin(vibPhase + 4 * PI));
      display.fillCircle(vibWaveRight, vibDotY, 2, SSD1306_WHITE);

    } else if (mode == 8) {
      display.setTextSize(1);
      display.setCursor(2, 4);
      display.print("BITCRUSH");
      drawLoopIcon(SCREEN_WIDTH - 18, 2);
      drawOnOffIndicator(bitcrushOn);

      display.setTextSize(2);
      display.setCursor(4, BLUE_TOP + 2);
      display.print(bitcrushKnob);
      display.print("%");

      const int BITCRUSH_MIN_DOTS = 14;
      const int BITCRUSH_MAX_DOTS = 30;
      const int BITCRUSH_MAX_RADIUS = 20;
      const int BITCRUSH_MIN_RADIUS = 6;
      const int bitCenterX = SCREEN_WIDTH / 2;
      const int bitCenterY = BLUE_TOP + 26;
      const unsigned long BITCRUSH_FLIP_INTERVAL_MS = 250;

      if (now - bitcrushStaticLastFlipTime >= BITCRUSH_FLIP_INTERVAL_MS) {
        bitcrushStaticFrameIndex = (bitcrushStaticFrameIndex + 1) % 4;
        bitcrushStaticLastFlipTime = now;
      }

      int bitRadius = BITCRUSH_MAX_RADIUS - (int)((long)bitcrushKnob * (BITCRUSH_MAX_RADIUS - BITCRUSH_MIN_RADIUS) / 100);
      int bitDotCount = BITCRUSH_MIN_DOTS + (int)((long)bitcrushKnob * (BITCRUSH_MAX_DOTS - BITCRUSH_MIN_DOTS) / 100);

      long bitLcgState = (long)bitcrushStaticFrameIndex * 7919L + 1L;
      int bitDrawn = 0;
      int bitAttempts = 0;
      int bitMaxAttempts = bitDotCount * 6;

      while (bitDrawn < bitDotCount && bitAttempts < bitMaxAttempts) {
        bitAttempts++;
        bitLcgState = bitLcgState * 1103515245L + 12345L;
        long bitRawX = bitLcgState % (long)(bitRadius * 2 + 1);
        if (bitRawX < 0) bitRawX = -bitRawX;
        int bitRx = (int)bitRawX - bitRadius;

        bitLcgState = bitLcgState * 1103515245L + 12345L;
        long bitRawY = bitLcgState % (long)(bitRadius * 2 + 1);
        if (bitRawY < 0) bitRawY = -bitRawY;
        int bitRy = (int)bitRawY - bitRadius;

        if (bitRx * bitRx + bitRy * bitRy <= bitRadius * bitRadius) {
          display.fillRect(bitCenterX + bitRx, bitCenterY + bitRy, 2, 2, SSD1306_WHITE);
          bitDrawn++;
        }
      }
    }

    display.display();
  }
}