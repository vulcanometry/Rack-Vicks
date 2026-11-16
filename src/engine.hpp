#pragma once
#include "dsp.hpp"

// The Vicks sound engine:
//   generator (one of nine algorithms, 2x oversampled)
//   -> filter section (multi-notch + FIELD low/high pass) -> VCA
//   -> processor (delay that morphs into a reverb, frequency shifter in the loop)

namespace vicks {

enum Algo {
	ALGO_ANALOG_PAIR,     // 2 analog-style oscillators, metallic processor
	ALGO_SHAPER_PAIR,     // waveshaping oscillator pair, diffuse processor
	ALGO_SINE_CLUSTER,    // multi-sine cluster bank, metallic
	ALGO_WT_CRUSH,        // 4 wavetable oscillators (bit reduction), metallic
	ALGO_WT_CHORUS,       // 4 wavetable oscillators, metallic
	ALGO_WT_FOLD,         // 4 wavetable oscillators (wavefolding), diffuse
	ALGO_CLUSTER_DIATONIC,// 4-osc diatonic-quantised cluster scanning, diffuse
	ALGO_CLUSTER_METAL,   // 4-osc cluster scanning, metallic
	ALGO_CLUSTER_DIFFUSE, // 4-osc cluster scanning, diffuse
	NUM_ALGOS
};

inline bool algoIsDiffuse(int algo) {
	return algo == ALGO_SHAPER_PAIR || algo == ALGO_WT_FOLD || algo == ALGO_CLUSTER_DIATONIC || algo == ALGO_CLUSTER_DIFFUSE;
}

inline const char* algoName(int algo) {
	static const char* names[NUM_ALGOS] = {
		"Analog pair / metallic",
		"Waveshaping pair / diffuse",
		"Sine cluster bank / metallic",
		"Crushed wavetables / metallic",
		"Wavetable quartet / metallic",
		"Folded wavetables / diffuse",
		"Diatonic clusters / diffuse",
		"Clusters / metallic",
		"Clusters / diffuse",
	};
	return names[clamp(algo, 0, NUM_ALGOS - 1)];
}

// Values from the panel after CV, normalised. Bipolar controls are -1..1
// with 0 at 12:00.
struct GenControls {
	int algo = 0;
	float f0 = 65.f;    // BASIS in Hz
	float warp = 0.f;   // -1..1
	float span = 0.f;   // -1..1
	float morph = 0.5f; // 0..1
	float fuse = 0.f;   // -1..1
	float scan = 0.f;   // 0..1
	float seed = 0.f;   // 0..1
};

// Per-algorithm side effects of FUSE / WARP that live outside the generator
struct FxSettings {
	float chorus = 0.f;     // chorus mix
	float filterRes = 0.f;  // 0..1
	float filterShape = 0.f;// drive into the filter
	float bandpass = 0.f;   // LP -> BP morph
	float sineLevel = 0.f;  // parallel sine after the filter
	float audioMod = 0.f;   // generator -> processor audio-rate modulation
	float notch = 0.f;      // multi-notch depth
};

inline FxSettings fxFor(const GenControls& c) {
	FxSettings fx;
	const float ccw = std::max(-c.fuse, 0.f);
	const float cw = std::max(c.fuse, 0.f);
	switch (c.algo) {
		case ALGO_ANALOG_PAIR:
			fx.chorus = ccw;
			fx.filterRes = cw;
			fx.filterShape = cw;
			fx.notch = 0.8f;
			break;
		case ALGO_SHAPER_PAIR:
			fx.audioMod = ccw;
			fx.notch = 0.8f;
			break;
		case ALGO_SINE_CLUSTER:
			fx.chorus = ccw;
			break;
		case ALGO_WT_CRUSH:
			fx.bandpass = ccw;
			fx.sineLevel = ccw;
			fx.chorus = cw;
			fx.filterRes = cw;
			fx.filterShape = cw;
			break;
		case ALGO_WT_CHORUS:
			fx.chorus = std::max(ccw, cw);
			fx.filterRes = cw;
			break;
		case ALGO_WT_FOLD:
			fx.audioMod = ccw;
			break;
		default:
			// Cluster algorithms: stereo chorus CCW, saturation (inside the generator) CW
			fx.chorus = ccw;
			break;
	}
	return fx;
}

// Equal-power pan, unity gain at centre
inline void panGains(float pan, float* gl, float* gr) {
	float a = (clamp(pan, -1.f, 1.f) + 1.f) * float(M_PI) / 4.f;
	*gl = std::cos(a) * float(M_SQRT2);
	*gr = std::sin(a) * float(M_SQRT2);
}


struct Generator {
	static const int MAX_OSC = 6;
	static const int POSITIONS = 8; // cluster scan positions per oscillator

	int algo = 0;
	int nOsc = 4;

	// Phases: [osc][slot]. Cluster algorithms crossfade two slots per oscillator.
	float phase[MAX_OSC][2] = {};
	float dt[MAX_OSC][2] = {};
	float amp[MAX_OSC][2] = {};
	float frame[MAX_OSC][2] = {};
	int mip[MAX_OSC][2] = {};
	float gl[MAX_OSC] = {}, gr[MAX_OSC] = {};
	float prevOut[MAX_OSC] = {};
	float fbHist[MAX_OSC] = {};

	// Wavetable set crossfade (SCAN)
	int setA = 0, setB = 1;
	float setX = 0.f;

	// Modulation amounts derived from WARP / FUSE / SCAN
	float pm = 0.f, xm = 0.f, phaseFb = 0.f, crush = 0.f, ring = 0.f;
	float noiseMod = 0.f, foldAmt = 0.f, satAmt = 0.f;
	float modFb = 0.f, carFb = 0.f, index = 0.f, drive = 1.f;
	float morphScan = 0.f, modScan = 0.f;
	float shaperW[5] = {1.f, 0.f, 0.f, 0.f, 0.f};

	Rng rng;
	float noiseLp = 0.f;

	void reset() {
		for (int i = 0; i < MAX_OSC; i++) {
			for (int s = 0; s < 2; s++)
				phase[i][s] = hashUnit(77u, i * 2 + s);
			prevOut[i] = 0.f;
			fbHist[i] = 0.f;
		}
	}

	// Called at control rate. fs2 is the oversampled rate.
	void control(const GenControls& c, float fs2) {
		algo = c.algo;
		const float seedPos = c.seed * 48.f;
		const float span = c.span;
		const float aspan = std::fabs(span);
		const float wccw = std::max(-c.warp, 0.f);
		const float wcw = std::max(c.warp, 0.f);
		const float fcw = std::max(c.fuse, 0.f);
		const float maxDt = 0.45f;
		auto toDt = [&](float hz) {
			return clamp(hz / fs2, 0.f, maxDt);
		};

		pm = xm = phaseFb = crush = ring = noiseMod = foldAmt = satAmt = 0.f;
		modFb = carFb = index = 0.f;
		for (int i = 0; i < MAX_OSC; i++)
			for (int s = 0; s < 2; s++)
				amp[i][s] = 0.f;

		// Wavetable set from SCAN (random sets, smoothly crossfaded)
		{
			float sp = c.scan * (WaveBank::SETS - 1);
			setA = clamp((int) sp, 0, WaveBank::SETS - 2);
			setB = setA + 1;
			setX = sp - setA;
		}

		switch (algo) {
			case ALGO_ANALOG_PAIR: {
				nOsc = 2;
				// Osc 0 = carrier, osc 1 = modulator (relative tuning +/- 2 octaves)
				dt[0][0] = toDt(c.f0);
				dt[1][0] = toDt(c.f0 * std::pow(2.f, span * 2.f));
				xm = wccw;
				ring = wcw;
				morphScan = c.morph;
				modScan = c.scan;
				panGains(-0.15f, &gl[0], &gr[0]);
				panGains(0.35f, &gl[1], &gr[1]);
			} break;

			case ALGO_SHAPER_PAIR: {
				nOsc = 2;
				// Osc 0 = sine modulator; carriers for left (slot 0) and right (slot 1)
				float ratio = std::pow(2.f, aspan * 2.f);
				float stereo = std::max(span, 0.f);
				dt[0][0] = toDt(c.f0);
				dt[1][0] = toDt(c.f0 * ratio * (1.f - 0.006f * stereo));
				dt[1][1] = toDt(c.f0 * ratio * (1.f + 0.006f * stereo));
				index = std::fabs(c.warp) * 0.7f;
				modFb = wccw * 0.22f;
				carFb = wcw * 0.22f;
				drive = 1.f + c.morph * 5.f;
				satAmt = fcw;
				// Shaper blend from SCAN
				float sum = 0.f;
				for (int k = 0; k < 5; k++) {
					float w = scanRand(101u + k, c.scan * 24.f);
					w = w * w * w;
					shaperW[k] = w;
					sum += w;
				}
				for (int k = 0; k < 5; k++)
					shaperW[k] /= sum;
			} break;

			case ALGO_SINE_CLUSTER: {
				nOsc = 6;
				phaseFb = wccw * 0.3f;
				xm = wcw;
				noiseMod = fcw;
				ring = c.scan;
				clusterSlots(c, seedPos, fs2, false, false);
				float stereo = std::max(span, 0.f);
				for (int i = 0; i < nOsc; i++) {
					float pan = ((i % 2) ? 1.f : -1.f) * (0.3f + 0.7f * i / (nOsc - 1)) * stereo;
					panGains(pan, &gl[i], &gr[i]);
				}
			} break;

			case ALGO_WT_CRUSH:
			case ALGO_WT_CHORUS:
			case ALGO_WT_FOLD: {
				nOsc = 4;
				if (algo == ALGO_WT_CRUSH)
					crush = wccw;
				else
					pm = wccw;
				xm = wcw;
				if (algo == ALGO_WT_FOLD)
					foldAmt = fcw;
				const float waveSpread = (span < 0.f) ? aspan : 0.f;
				const float stereo = std::max(span, 0.f);
				for (int i = 0; i < nOsc; i++) {
					// SEED: pitch and wave offsets per oscillator
					float semis = (scanRand(200u + i, seedPos) * 2.f - 1.f) * 12.f;
					if (i == 0)
						semis *= 0.25f;
					float wOff = scanRand(300u + i, seedPos) * 2.f - 1.f;
					float detune = 1.f + 0.0015f * (i - 1.5f);
					dt[i][0] = toDt(c.f0 * detune * std::pow(2.f, aspan * semis / 12.f));
					amp[i][0] = 1.f;
					frame[i][0] = clamp((c.morph + waveSpread * wOff * 0.5f) * (WaveBank::FRAMES - 1), 0.f, (float) (WaveBank::FRAMES - 1));
					mip[i][0] = WaveBank::mipFor(dt[i][0]);
					static const float pans[4] = {-1.f, 1.f, -0.4f, 0.4f};
					panGains(pans[i] * stereo, &gl[i], &gr[i]);
				}
			} break;

			default: {
				// Cluster scanning algorithms
				nOsc = 4;
				pm = wccw;
				xm = wcw;
				satAmt = fcw;
				clusterSlots(c, seedPos, fs2, true, algo == ALGO_CLUSTER_DIATONIC);
				const float stereo = std::max(span, 0.f);
				static const float pans[4] = {-1.f, 1.f, -0.4f, 0.4f};
				for (int i = 0; i < nOsc; i++)
					panGains(pans[i] * stereo, &gl[i], &gr[i]);
			} break;
		}
	}

	// Cluster scan: each oscillator owns POSITIONS (frequency, wave) pairs chosen by
	// SEED. MORPH crossfades between neighbouring positions; even positions play on
	// slot 0 and odd ones on slot 1 so the incoming position always starts silent.
	void clusterSlots(const GenControls& c, float seedPos, float fs2, bool wavetable, bool diatonic) {
		const float pos = c.morph * (POSITIONS - 1);
		int n = std::min((int) pos, POSITIONS - 2);
		float x = pos - n;
		for (int i = 0; i < nOsc; i++) {
			for (int k = 0; k < 2; k++) {
				const int p = n + k;
				const int slot = p & 1;
				const uint32_t key = 1000u + i * 64u + p;
				float ratio;
				if (wavetable) {
					// Bipolar spread: SPAN scales (and CCW mirrors) the offsets
					float oct = (scanRand(key, seedPos) * 2.f - 1.f) * 1.5f;
					float semis = oct * 12.f * c.span;
					if (diatonic) {
						static const int scale[7] = {0, 2, 4, 5, 7, 9, 11};
						int st = (int) std::round(semis);
						int octave = (int) std::floor(st / 12.f);
						int deg = st - octave * 12;
						int best = 0;
						for (int d = 0; d < 7; d++)
							if (std::abs(scale[d] - deg) < std::abs(best - deg))
								best = scale[d];
						semis = (float) (octave * 12 + best);
					}
					ratio = std::pow(2.f, semis / 12.f);
					frame[i][slot] = scanRand(key + 50000u, seedPos) * (WaveBank::FRAMES - 1);
				}
				else {
					// Sine clusters: inharmonic ratios from SEED, SPAN adds spread
					float oct = scanRand(key, seedPos) * 2.5f - 0.5f;
					float spread = std::fabs(c.span) * ((float) i / (nOsc - 1) - 0.5f) * 1.5f;
					ratio = std::pow(2.f, oct + spread);
				}
				float level = wavetable ? 1.f : 0.4f + 0.6f * scanRand(key + 90000u, seedPos);
				dt[i][slot] = clamp(c.f0 * ratio / fs2, 0.f, 0.45f);
				mip[i][slot] = WaveBank::mipFor(dt[i][slot]);
				float w = k ? x : 1.f - x;
				amp[i][slot] = std::sin(w * float(M_PI) / 2.f) * level;
			}
		}
	}

	float wave(int i, int slot, float ph) const {
		const WaveBank& wb = WaveBank::get();
		float fp = frame[i][slot];
		int f0 = std::min((int) fp, WaveBank::FRAMES - 2);
		float fx = fp - f0;
		int m = mip[i][slot];
		float a = readTable(wb.table(setA, f0, m), ph);
		float b = readTable(wb.table(setA, f0 + 1, m), ph);
		float v = a + (b - a) * fx;
		if (setX > 1e-3f) {
			float a2 = readTable(wb.table(setB, f0, m), ph);
			float b2 = readTable(wb.table(setB, f0 + 1, m), ph);
			float v2 = a2 + (b2 - a2) * fx;
			v += (v2 - v) * setX;
		}
		return v;
	}

	// Pulse -> saw -> sine as x goes 0 -> 0.5 -> 1
	static float analogWave(float ph, float dt, float x) {
		float sq = (ph < 0.5f ? 1.f : -1.f) + polyBlep(ph, dt) - polyBlep(wrap01(ph + 0.5f), dt);
		float saw = 2.f * ph - 1.f - polyBlep(ph, dt);
		float sn = sin2pi(ph);
		if (x < 0.5f)
			return sq + (saw - sq) * (x * 2.f);
		return saw + (sn - saw) * (x * 2.f - 1.f);
	}

	// One oversampled sample
	void process(float* outL, float* outR) {
		float l = 0.f, r = 0.f;
		switch (algo) {
			case ALGO_ANALOG_PAIR: {
				float d1 = dt[1][0];
				float mod = analogWave(phase[1][0], d1, modScan);
				phase[1][0] = wrap01(phase[1][0] + d1);
				float d0 = clamp(dt[0][0] * dsp::exp2_taylor5(xm * 3.f * mod), 0.f, 0.45f);
				float car = analogWave(phase[0][0], d0, morphScan);
				phase[0][0] = wrap01(phase[0][0] + d0);
				float ringOut = car * mod * 1.4f;
				float a = car * (1.f - ring) + ringOut * ring;
				float b = mod * 0.5f * (1.f - ring);
				l = (a * gl[0] + b * gl[1]) * 0.25f;
				r = (a * gr[0] + b * gr[1]) * 0.25f;
			} break;

			case ALGO_SHAPER_PAIR: {
				// Modulator with self feedback (averaged to tame the feedback squeal)
				float m = sin2pi(phase[0][0] + modFb * fbHist[0]);
				fbHist[0] = 0.5f * (fbHist[0] + m);
				phase[0][0] = wrap01(phase[0][0] + dt[0][0]);
				float out[2];
				for (int s = 0; s < 2; s++) {
					float c = sin2pi(phase[1][s] + index * m + carFb * prevOut[s]);
					prevOut[s] = 0.5f * (prevOut[s] + c);
					phase[1][s] = wrap01(phase[1][s] + dt[1][s]);
					float y = c * drive;
					float fy = fold(y);
					float shaped = shaperW[0] * fy
						+ shaperW[1] * sin2pi(y * 0.25f)
						+ shaperW[2] * fastTanh(y)
						+ shaperW[3] * fy * (4.f * fy * fy - 3.f)
						+ shaperW[4] * (2.f * std::fabs(fy) - 1.f);
					if (satAmt > 0.f)
						shaped += (fastTanh(shaped * (1.f + 8.f * satAmt)) - shaped) * satAmt;
					out[s] = shaped * 0.9f;
				}
				l = out[0];
				r = out[1];
			} break;

			case ALGO_SINE_CLUSTER: {
				float nz = 0.f;
				if (noiseMod > 0.f) {
					noiseLp += (rng.uni() - noiseLp) * 0.08f;
					nz = noiseLp * noiseMod * 3.f;
				}
				float o[MAX_OSC];
				for (int i = 0; i < nOsc; i++) {
					float sum = 0.f;
					float xmod = dsp::exp2_taylor5(xm * 2.f * prevOut[(i + nOsc - 1) % nOsc] + nz);
					for (int s = 0; s < 2; s++) {
						if (amp[i][s] <= 0.f)
							continue;
						float d = clamp(dt[i][s] * xmod, 0.f, 0.45f);
						sum += amp[i][s] * sin2pi(phase[i][s] + phaseFb * fbHist[i]);
						phase[i][s] = wrap01(phase[i][s] + d);
					}
					fbHist[i] = 0.5f * (fbHist[i] + sum);
					o[i] = sum;
				}
				for (int i = 0; i < nOsc; i++)
					prevOut[i] = o[i];
				for (int i = 0; i < nOsc; i += 2) {
					// SCAN: crossfade each pair from sum to ring modulation
					float sum0 = o[i], sum1 = o[i + 1];
					float rm = o[i] * o[i + 1] * 2.f;
					float a = sum0 * (1.f - ring) + rm * ring * 0.5f;
					float b = sum1 * (1.f - ring) + rm * ring * 0.5f;
					l += a * gl[i] + b * gl[i + 1];
					r += a * gr[i] + b * gr[i + 1];
				}
				l *= 0.3f;
				r *= 0.3f;
			} break;

			default: {
				// Wavetable and cluster algorithms (4 oscillators)
				float o[MAX_OSC];
				for (int i = 0; i < nOsc; i++) {
					float prev = prevOut[(i + nOsc - 1) % nOsc];
					float xmod = (xm > 0.f) ? dsp::exp2_taylor5(xm * 2.5f * prev) : 1.f;
					float pmo = pm * 0.35f * prev;
					float sum = 0.f;
					for (int s = 0; s < 2; s++) {
						if (amp[i][s] <= 0.f)
							continue;
						float d = clamp(dt[i][s] * xmod, 0.f, 0.45f);
						sum += amp[i][s] * wave(i, s, wrap01(phase[i][s] + pmo));
						phase[i][s] = wrap01(phase[i][s] + d);
					}
					if (crush > 0.f) {
						// Continuous bit depth: 12 bits down to 1
						float levels = std::pow(2.f, 11.f - 10.5f * std::sqrt(crush));
						sum = std::round(sum * levels) / levels;
					}
					if (foldAmt > 0.f)
						sum = fold(sum * (1.f + 4.f * foldAmt));
					o[i] = sum;
				}
				for (int i = 0; i < nOsc; i++) {
					prevOut[i] = o[i];
					l += o[i] * gl[i];
					r += o[i] * gr[i];
				}
				l *= 0.6f;
				r *= 0.6f;
				if (satAmt > 0.f) {
					float g = 1.f + 9.f * satAmt;
					l += (fastTanh(l * g) - l) * satAmt;
					r += (fastTanh(r * g) - r) * satAmt;
				}
			} break;
		}
		*outL = l;
		*outR = r;
	}
};


// Stereo chorus: two modulated taps per side, quadrature LFOs
struct Chorus {
	DelayLine dl[2];
	float ph[2] = {0.f, 0.37f};

	void init(float fs) {
		dl[0].init((int) (fs * 0.05f) + 8);
		dl[1].init((int) (fs * 0.05f) + 8);
	}
	void process(float* l, float* r, float mix, float fs) {
		ph[0] = wrap01(ph[0] + 0.27f / fs);
		ph[1] = wrap01(ph[1] + 0.41f / fs);
		float in[2] = {*l, *r};
		float out[2];
		for (int c = 0; c < 2; c++) {
			float off = c * 0.25f;
			float d1 = fs * (0.009f + 0.004f * (1.f + sin2pi(ph[0] + off)));
			float d2 = fs * (0.014f + 0.005f * (1.f + sin2pi(ph[1] + off + 0.5f)));
			float wet = 0.5f * (dl[c].readLinear(d1) + dl[c].readLinear(d2));
			dl[c].write(in[c]);
			out[c] = in[c] * (1.f - 0.4f * mix) + wet * mix;
		}
		*l = out[0];
		*r = out[1];
	}
};


// FIELD filter section: optional multi-notch, low pass below 12:00, high pass above,
// with resonance / drive / bandpass morph supplied by the algorithm.
struct FilterSection {
	Svf notch[2][4];
	Svf lp[2], hp[2];
	float notchDepth = 0.f;
	float shape = 0.f, bandpass = 0.f;

	void control(float field, const FxSettings& fx, const float* notchHz, float fs) {
		notchDepth = fx.notch;
		shape = fx.filterShape;
		bandpass = fx.bandpass;
		// Low pass: 20 kHz at 12:00 closing to ~40 Hz fully CCW, finer resolution near the top
		float x = std::max(-field, 0.f);
		float lpHz = 20000.f * std::pow(2.f, -9.f * std::pow(x, 1.5f));
		// High pass: 12 Hz at 12:00 opening to ~6 kHz fully CW
		float y = std::max(field, 0.f);
		float hpHz = 12.f * std::pow(2.f, 9.f * std::pow(y, 1.2f));
		// Bandpass morph pulls the centre into the audible middle
		float bpHz = 60.f * std::pow(2.f, 7.f * (field * 0.5f + 0.5f));
		float fc = std::exp(std::log(lpHz) + (std::log(bpHz) - std::log(lpHz)) * bandpass);
		float q = 0.65f + fx.filterRes * fx.filterRes * 12.f + bandpass * 1.5f;
		for (int c = 0; c < 2; c++) {
			lp[c].set(std::min(fc, 0.45f * fs) / fs, q);
			hp[c].set(hpHz / fs, 0.7f);
			for (int k = 0; k < 4; k++)
				notch[c][k].set(notchHz[k] * (c ? 1.06f : 1.f) / fs, 1.f);
		}
	}

	float process(int c, float x) {
		if (notchDepth > 0.f) {
			float ap = x;
			for (int k = 0; k < 4; k++)
				ap = notch[c][k].allpass(ap);
			x += (ap - x) * 0.5f * notchDepth;
		}
		if (shape > 0.f)
			x = fastTanh(x * (1.f + 3.f * shape)) / (1.f + 1.2f * shape);
		lp[c].process(x);
		float y = lp[c].lp + (lp[c].bp * lp[c].k - lp[c].lp) * bandpass;
		hp[c].process(y);
		return hp[c].hp;
	}
};


struct ProcControls {
	float timeSamples = 4800.f;
	float form = 0.f;     // 0..1
	float feed = 0.f;     // -1..1
	float shiftHz = 0.f;
	float audioMod = 0.f; // depth of generator -> delay modulation
	float tapRatio[4] = {1.f, 0.5f, 0.3f, 0.7f};
	bool diffuse = false;
};

// Delay-based processor. FORM moves it from distinct repeats (one tap, no
// diffusion) towards a reverb (more taps, in-loop allpass diffusion,
// modulation). A frequency shifter sits in the feedback loop.
struct Processor {
	DelayLine main[2];
	Diffuser diff[2][4];
	Hilbert hilbert[2];
	OnePole damp[2];
	DcBlock dc[2];
	float tSm = 4800.f;
	float diffLen[2][4] = {};
	float lfoPh[4] = {0.f, 0.25f, 0.5f, 0.75f};
	float shiftPh = 0.f;
	float fs = 48000.f;

	void init(float sampleRate) {
		fs = sampleRate;
		for (int c = 0; c < 2; c++) {
			main[c].init((int) (fs * 2.2f) + 16);
			for (int j = 0; j < 4; j++) {
				diff[c][j].dl.init((int) (fs * 0.2f) + 8);
				diffLen[c][j] = fs * 0.002f;
			}
		}
	}
	void clear() {
		for (int c = 0; c < 2; c++) {
			main[c].clear();
			for (int j = 0; j < 4; j++)
				diff[c][j].dl.clear();
		}
	}

	void process(float inL, float inR, float modSig, const ProcControls& p, float* outL, float* outR) {
		const float maxD = (float) main[0].size() - 8.f;
		// Slow glide on the delay time: turning TIME bends repeats like tape
		tSm += (clamp(p.timeSamples, 4.f, fs * 2.05f) - tSm) * (1.f / (0.08f * fs));

		const float form = p.form;
		// Tap weights: more taps fade in as FORM rises
		float w[4];
		w[0] = 1.f;
		float norm = 1.f;
		for (int k = 1; k < 4; k++) {
			w[k] = clamp(form * 3.f - (k - 1), 0.f, 1.f) * 0.8f;
			norm += w[k] * w[k];
		}
		norm = 1.f / std::sqrt(norm);

		static const float lfoRate[4] = {0.13f, 0.21f, 0.37f, 0.53f};
		float lfo[4];
		for (int k = 0; k < 4; k++) {
			lfoPh[k] = wrap01(lfoPh[k] + lfoRate[k] / fs);
			lfo[k] = sin2pi(lfoPh[k]);
		}
		const float modDepth = fs * (0.0002f + form * form * (p.diffuse ? 0.005f : 0.0018f));
		const float am = modSig * p.audioMod * fs * 0.0025f;

		shiftPh = wrap01(shiftPh + p.shiftHz / fs);
		const float cosA = sin2pi(shiftPh + 0.25f), sinA = sin2pi(shiftPh);
		const float cosB = sin2pi(shiftPh + 0.5f), sinB = sin2pi(shiftPh + 0.25f); // +90 degrees on the right
		const float mixType = std::max(p.feed, 0.f);

		float loop[2];
		for (int c = 0; c < 2; c++) {
			float sum = 0.f;
			for (int k = 0; k < 4; k++) {
				if (w[k] <= 0.f)
					continue;
				float l = lfo[(k + 2 * c) & 3];
				float d = tSm * p.tapRatio[k] + modDepth * (1.f + l) + am * (k + 1) * 0.5f + 2.f;
				sum += w[k] * main[c].readHermite(clamp(d, 2.f, maxD));
			}
			sum *= norm;
			float re, im;
			hilbert[c].process(sum, &re, &im);
			float shifted = c ? (re * cosB - im * sinB) : (re * cosA - im * sinA);
			// FEED CCW: shifter output alone. CW: mixed with the raw delay (moving notches)
			loop[c] = shifted * (1.f - 0.5f * mixType) + sum * 0.5f * mixType;
		}

		// Feedback path, cross-coupled for the diffuse structure
		float fb[2] = {p.feed * loop[0], p.feed * loop[1]};
		if (p.diffuse) {
			float a = float(M_PI) * (0.15f + 0.2f * form);
			float ca = std::cos(a), sa = std::sin(a);
			float l = fb[0], r = fb[1];
			fb[0] = ca * l - sa * r;
			fb[1] = sa * l + ca * r;
		}
		else {
			float l = fb[0], r = fb[1];
			fb[0] = 0.92f * l + 0.08f * r;
			fb[1] = 0.92f * r + 0.08f * l;
		}

		const float dampCoef = 1.f - std::exp(-2.f * float(M_PI) * (p.diffuse ? 6000.f : 9500.f) / fs);
		const float g = form * (p.diffuse ? 0.62f : 0.72f);
		static const float metalMs[4] = {1.13f, 1.79f, 2.71f, 3.97f};
		static const float diffRatio[4] = {0.043f, 0.071f, 0.113f, 0.173f};
		const float in[2] = {inL, inR};
		// Diffusion fades in over the bottom of the FORM range, so the extra
		// allpass delay never switches in abruptly
		const float diffMix = clamp(form * 4.f, 0.f, 1.f);
		for (int c = 0; c < 2; c++) {
			float x = damp[c].process(fb[c], dampCoef);
			x = dc[c].process(x) + in[c];
			float dx = x;
			for (int j = 0; j < 4; j++) {
				float target;
				if (p.diffuse)
					target = clamp(tSm * diffRatio[j], fs * 0.0015f, fs * 0.18f);
				else
					target = fs * metalMs[j] * 0.001f;
				target *= c ? 1.09f : 1.f;
				diffLen[c][j] += (target - diffLen[c][j]) * 0.0005f;
				float d = diffLen[c][j] + (p.diffuse ? am * 0.3f : 0.f);
				d = clamp(d, 1.f, (float) diff[c][j].dl.size() - 4.f);
				dx = diff[c][j].process(dx, d, g);
			}
			x += (dx - x) * diffMix;
			main[c].write(softClip(x, 1.6f));
		}
		*outL = loop[0];
		*outR = loop[1];
	}
};

} // namespace vicks
