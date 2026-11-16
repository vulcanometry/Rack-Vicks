# Vicks

A VCV Rack 2 plugin containing one module, **Vicks**: an experimental stereo drone voice and effects
processor. It is a 24 HP module.

Two 3-way toggles select one of nine algorithms. Each algorithm runs a **generator** (an oscillator pair,
a sine cluster bank or four wavetable oscillators) through a filter section and a VCA, and then into a
**processor**: a delay that turns into a reverb as you raise FORM, with a frequency shifter in its feedback
loop. External stereo audio can be mixed into the generator, so Vicks also works as an effects processor.

## Signal path

```
            EXT IN L/R ─┐ (input volume)
GENERATOR ──────────────┴─► chorus ─► multi-notch ─► FIELD LP / HP ─► VCA ─┬──────────────── dry ─┐
(algorithm, 2x oversampled)                                                 └─► PROCESSOR ── wet ──┴─ MIX ─► output volume ─► OUT L/R
                                                                                 ▲ TIME, FORM, FEED, CELL, SEED
```

## Panel

| Area | Controls |
|---|---|
| Top left (generator) | **WARP** (cross-modulation), **SPAN** (pitch / stereo spread), left logo LED |
| Top centre | Two algorithm toggles with three bank LEDs between them; SEED and SCAN light windows; **MODE** button |
| Top right (processor) | **MIX** (generator / processor), **FEED** (bipolar feedback), right logo LED |
| Middle | **MORPH** (large), the two endless **encoders**, the **BANK / VOL** button, **FORM** (large); BASIS and TIME light windows under the encoders |
| Lower | **FUSE**, MORPH attenuverter, **FIELD** (large), FORM attenuverter, **CELL**; the BASIS, FIELD and TIME attenuverters below FIELD |
| Jacks, upper row | WARP, SPAN, FUSE (left); CELL, MIX, FEED (right) |
| Jacks, lower row | IN L, IN R, VCA; MORPH, BASIS, FIELD, TIME, FORM (attenuverted); SCAN; OUT L, OUT R |

Level meters above the audio input and output jacks show the left channel in white and the right in red.

### Parameters

WARP, SPAN, MIX, FEED, FUSE and FIELD are bipolar: 12:00 is their neutral position.

| Parameter | Function |
|---|---|
| WARP | Generator cross-modulation. CCW and CW give two different types (see the algorithm table). |
| SPAN | Oscillator spread: pitch, wave and stereo spreading, depending on the algorithm. |
| MORPH | Timbre scan: waveform, wavetable or cluster position. |
| FUSE | Algorithm-dependent timbral effect: chorus, filter resonance, saturation, folding and so on. |
| FIELD | Filter. CCW closes a low pass; CW opens a high pass. 12:00 leaves the signal unfiltered. |
| BASIS (encoder) | Root frequency, 8 octaves from C-1. The BASIS jack tracks 1V/oct with its attenuverter fully CW. |
| TIME (encoder) | Scales every delay length in the processor, from 2 ms to 2 s. Changes glide, like tape. |
| SEED (encoder) | Manually scans internal random values: delay tap spacing, oscillator pitch and wave offsets, and notch frequencies. There is no SEED jack. |
| SCAN (encoder) | Scans random timbral sets: waveshapes, wavetable sets, the modulator wave or the ring-mod mix. |
| FORM | Processor structure. CCW gives clean, distinct repeats. Turning it CW adds taps, allpass diffusion and modulation until the repeats become a reverb. |
| FEED | Processor feedback. 12:00 is no feedback, and either extreme sustains almost indefinitely. FEED also sets how the frequency shifter is mixed into the loop: CCW uses the shifter's output alone, while CW mixes it with the raw delay for a moving multi-notch effect. |
| CELL | Amount of frequency shift in the feedback loop, from 0 to about 1.2 kHz. |
| MIX | Equal-power blend of the dry generator and the processor. |

The CV jacks expect 0–5 V for a control's full range. The jacks for WARP, SPAN, FUSE, CELL, MIX, FEED and SCAN add to their control. The five jacks in the bottom centre (MORPH, BASIS, FIELD, TIME, FORM) go through their attenuverters.

## Algorithms

The left toggle selects the row and the right toggle the column (up, middle, down). Hover over a toggle
to see the current algorithm's name.

| # | Left | Right | Generator | Processor | WARP CCW / CW | SPAN | FUSE CCW / CW | SCAN |
|---|---|---|---|---|---|---|---|---|
| 1 | up | up | Analog pair (carrier + modulator) | metallic | exponential FM / ring mod | modulator tuning ±2 oct | chorus / filter resonance + drive | modulator wave (pulse–saw–sine) |
| 2 | up | mid | Waveshaping oscillator pair | diffuse | FM index + modulator feedback / FM index + carrier feedback | carrier ratio (CW adds stereo detune) | audio-rate delay modulation / saturation | waveshaper set |
| 3 | up | down | 6-sine cluster bank | metallic | phase feedback / exponential cross-FM | frequency spread (CW adds stereo) | chorus / noise FM | ring-mod mix |
| 4 | mid | up | 4 wavetable oscillators | metallic | bit-depth reduction / exponential cross-FM | CCW pitch + wave spread, CW pitch + stereo spread | bandpass morph + parallel sine / chorus + resonance + drive | wavetable set |
| 5 | mid | mid | 4 wavetable oscillators | metallic | phase modulation / exponential cross-FM | as 4 | chorus / chorus + resonance | wavetable set |
| 6 | mid | down | 4 wavetable oscillators | diffuse | phase modulation / exponential cross-FM | as 4 | audio-rate allpass + delay modulation / wavefolding | wavetable set |
| 7 | down | up | 4-osc diatonic cluster scanning | diffuse | phase modulation / exponential cross-FM | bipolar cluster spread, quantised to a major scale (CW adds stereo) | stereo chorus / saturation | wavetable set |
| 8 | down | mid | 4-osc cluster scanning | metallic | phase modulation / exponential cross-FM | bipolar cluster spread (CW adds stereo) | stereo chorus / saturation | wavetable set |
| 9 | down | down | 4-osc cluster scanning | diffuse | phase modulation / exponential cross-FM | as 8 | stereo chorus / saturation | wavetable set |

**Cluster scanning** works like a wavetable scan, except that each scan position pairs a wave with its own frequency offset, and each oscillator has different pairs. MORPH crossfades smoothly between neighbouring positions, and SEED re-rolls the pairs.

The **metallic** processor uses short, fixed allpass diffusers that ring. The **diffuse** processor scales its diffusers with TIME and cross-couples the two channels.

For all algorithms, the VCA sits after the filter section and before the processor.

## Buttons and modes

| Action | Result |
|---|---|
| MODE | Switches the encoders between **BASIS + TIME** and **SEED + SCAN**. The active labels are white and the inactive ones red. |
| Bottom button, in BASIS + TIME | **VOL** mode: the left encoder sets input volume (silence to +30 dB) and the right sets output volume (silence to +12 dB). The BASIS and TIME windows show the volumes, and the SEED and SCAN windows light when the input or output clips. Press again to leave. |
| Bottom button, in SEED + SCAN | **BANK**. There is one bank of nine algorithms, so the bank LEDs only flash. |
| MODE + bottom together | Randomises SEED and SCAN. |
| Hold MODE 5 s | **BASIS/TIME memory select.** Both logo LEDs blink. Press MODE to choose global (the upper windows light) or the bottom button to choose per-algorithm (the lower windows light). Press both buttons to confirm. |
| Hold bottom 5 s | **BASIS calibration** (see below). |

Encoders are endless. Drag them like knobs (Ctrl-drag for fine adjustment) or scroll over them. Hover to see the current value. Right-click an encoder to type a value: Hz for BASIS, ms for TIME, dB for the volumes, and % for SEED and SCAN. The module's context menu also has sliders for all six encoder values.

## Normalling

- **Audio inputs:** IN L and IN R are normalled together, so a mono source can go into either one.
- **Outputs:** OUT L is normalled into OUT R. With only OUT R patched, it carries the mono sum.
- **VCA:** with nothing patched, the VCA input sits at +5 V, which is fully open. Voltages above 5 V overdrive the processor input.
- **CV:**
  - A cable in **VCA** is also normalled to MORPH, BASIS, FIELD, TIME and FORM, through their attenuverters.
  - A cable in **MORPH** is normalled to BASIS, FIELD, TIME and FORM instead, overriding the VCA normalling.
  - With nothing in VCA or MORPH, a small offset (0.25 V) is normalled to all five, so the attenuverters work as fine-tune controls.

## Saved state

SEED and SCAN are stored per algorithm, and the volumes are global. BASIS and TIME are stored either per
algorithm (the default) or globally. All of this is saved with the patch.

## BASIS calibration

Rack's voltages are exact, so calibration is only needed to correct an external pitch source.

1. Set the BASIS attenuverter fully CW. Hold the bottom button for 5 seconds; the BASIS window blinks.
2. Send 1 V to BASIS. The left logo LED lights when the voltage is in range. Press the **bottom** button.
3. Send 3 V to BASIS. The right logo LED lights when it is in range. Press **MODE**.
   An out-of-range voltage makes the bank LEDs blink and is ignored.
4. Optional: hold MODE for 5 s to replace the sound with a reference sawtooth, and again to turn it off.
5. Hold the bottom button for 5 s to reset to the defaults. Press both buttons to save and exit.

**Reset BASIS calibration** in the context menu also clears it.

## Building

Requires the [VCV Rack 2 SDK](https://vcvrack.com/manual/Building#Building-Rack-plugins). On
Windows, build from an MSYS2 MinGW 64-bit shell. The Makefile expects the SDK at `../Rack-SDK`
(override with `make RACK_DIR=/path/to/Rack-SDK`):

```sh
make -j8      # builds plugin.dll / .so / .dylib
make dist     # packages dist/Vicks-2.0.0-<arch>.vcvplugin
make install RACK_USER_DIR="C:/Users/utmos/AppData/Local/Rack2"  # copies the package into your Rack user folder
```

Rack extracts and loads the `.vcvplugin` on its next start.

## Implementation notes

- **Sources:** `src/dsp.hpp` holds the building blocks: filters, delay lines, the Hilbert transformer and
  the wavetable bank. `src/engine.hpp` holds the generator, chorus, filter section and processor.
  `src/Vicks.cpp` holds the module, the panel and the button logic.
- **Generator:** runs at 2× oversampling and is decimated with `dsp::Decimator`. The analog pair uses
  PolyBLEP waves.
- **Wavetables:** 16 sets of 8 frames, built additively when the plugin loads (about 30 ms). Each frame
  is stored as 10 mip levels, so the oscillators stay band-limited. SCAN crossfades between sets.
- **SEED and SCAN:** both are smooth value noise, so turning them glides between random states rather
  than jumping.
- **Processor:** delay taps use cubic Hermite interpolation. Allpass diffusers and an LFO-modulated
  multi-tap delay are cross-coupled for the diffuse algorithms, and the frequency shifter uses an IIR
  Hilbert transformer. The loop has damping, a DC blocker and a soft clipper.
- **Algorithm changes:** the generator fades over 4 ms, while the processor tails carry on.
- **CPU:** about 0.5 µs per sample on a modern desktop CPU (around 2–3% of one core at 48 kHz).
