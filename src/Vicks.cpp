#include "plugin.hpp"
#include "engine.hpp"

// Vicks: an experimental stereo drone voice and effects processor.
// Two 3-way toggles pick one of nine algorithms. Each runs a generator into a
// filter section, a VCA and a delay that FORM morphs into a reverb, with a
// frequency shifter in its feedback loop. Two endless encoders edit BASIS +
// TIME or SEED + SCAN (MODE button), or the input/output volumes (VOL).

using namespace vicks;

namespace {

const int CONTROL_RATE = 8;        // control updates every N samples
const float NORMAL_OFFSET = 0.25f; // volts normalled to the CV jacks when nothing is patched
const float HOLD_TIME = 5.f;       // seconds for the long-press functions
const float BASIS_LOW_HZ = 8.1758f;// BASIS encoder minimum (C-1); range is 8 octaves
const float BASIS_OCTAVES = 8.f;

float timeMs(float t) {
	// 2 ms .. 2 s
	return 2.f * std::pow(2.f, 10.f * t);
}

float volInDb(float v) {
	// silence at 0, then -40 dB .. +30 dB
	return -40.f + 70.f * v;
}

float volOutDb(float v) {
	// silence at 0, then -48 dB .. +12 dB
	return -48.f + 60.f * v;
}

float volGain(float v, float db) {
	if (v <= 0.f)
		return 0.f;
	float g = std::pow(10.f, db / 20.f);
	// Fade the last couple of percent to true silence
	return g * std::min(1.f, v * 50.f);
}

std::string noteName(float hz) {
	static const char* names[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
	float midi = 69.f + 12.f * std::log2(hz / 440.f);
	int n = (int) std::round(midi);
	int cents = (int) std::round((midi - n) * 100.f);
	std::string s = string::f("%s%d", names[((n % 12) + 12) % 12], (int) std::floor(n / 12.f) - 1);
	if (cents != 0)
		s += string::f(" %+dc", cents);
	return s;
}

} // namespace


struct Vicks : Module {
	enum ParamId {
		WARP_PARAM,
		SPAN_PARAM,
		MIX_PARAM,
		FEED_PARAM,
		MORPH_PARAM,
		FORM_PARAM,
		FUSE_PARAM,
		FIELD_PARAM,
		CELL_PARAM,
		MORPH_ATT_PARAM,
		BASIS_ATT_PARAM,
		FIELD_ATT_PARAM,
		TIME_ATT_PARAM,
		FORM_ATT_PARAM,
		ENC_L_PARAM,
		ENC_R_PARAM,
		ALGO_L_PARAM,
		ALGO_R_PARAM,
		MODE_PARAM,
		BANK_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		WARP_INPUT,
		SPAN_INPUT,
		FUSE_INPUT,
		IN_L_INPUT,
		IN_R_INPUT,
		VCA_INPUT,
		MORPH_INPUT,
		BASIS_INPUT,
		FIELD_INPUT,
		TIME_INPUT,
		FORM_INPUT,
		CELL_INPUT,
		MIX_INPUT,
		FEED_INPUT,
		SCAN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_L_OUTPUT,
		OUT_R_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		ENUMS(BANK_LIGHT, 3),
		VOL_LIGHT,
		ENUMS(LOGO_LIGHT, 2),
		LIGHTS_LEN
	};

	// Encoder targets
	enum Target {
		T_BASIS,
		T_TIME,
		T_SEED,
		T_SCAN,
		T_VOL_IN,
		T_VOL_OUT,
		NUM_TARGETS
	};
	enum EncMode {
		ENC_BASIS_TIME,
		ENC_SEED_SCAN,
	};
	enum UiMode {
		UI_NORMAL,
		UI_SAVE_SELECT,
		UI_CALIBRATE,
	};

	// Saved state. SEED and SCAN are per algorithm, volumes are global, and
	// BASIS / TIME are per algorithm or global depending on globalBasisTime.
	float seed[NUM_ALGOS], scan[NUM_ALGOS], basis[NUM_ALGOS], time[NUM_ALGOS];
	float basisGlobal = 0.375f, timeGlobal = 0.65f;
	float volIn = 4.f / 7.f, volOut = 0.8f;
	bool globalBasisTime = false;
	float calLow = 1.f, calHigh = 3.f;
	EncMode encMode = ENC_BASIS_TIME;
	bool volMode = false;

	// Engine
	int algo = 0;
	int pendingAlgo = 0;
	float fade = 1.f;
	Generator gen;
	dsp::Decimator<2, 8> decim[2];
	Chorus chorus;
	FilterSection filter;
	Processor proc;
	float sinePhase = 0.f;
	float sawPhase = 0.f;
	int controlCounter = 0;

	// Control-rate values (smoothed)
	GenControls gc;
	FxSettings fx;
	ProcControls pc;
	float sm[16] = {};
	bool smInit = false;
	float mixCos = 0.707f, mixSin = 0.707f;
	float gainIn = 1.f, gainOut = 1.f;

	// Encoders and buttons
	float encLast[2] = {};
	bool encInit = false;
	bool topDown = false, botDown = false, combo = false, topLong = false, botLong = false;
	float topTime = 0.f, botTime = 0.f;
	UiMode uiMode = UI_NORMAL;
	bool pendingGlobal = false;
	bool calSaw = false;
	float calLowPending = 1.f, calHighPending = 3.f;
	float errorTimer = 0.f, bankFlash = 0.f, blinkClock = 0.f;

	// Read by the panel widgets
	float uiWindow[4] = {};     // SEED, SCAN, BASIS, TIME window fill
	float uiWindowGlow[4] = {}; // window brightness
	int uiLabel[6] = {};        // SEED, SCAN, BASIS, TIME, BANK, VOL: 0 off, 1 red, 2 white
	float uiMeter[4] = {};      // in L, in R, out L, out R (0..1)
	float peak[4] = {};
	float clipIn = 0.f, clipOut = 0.f;

	struct EncoderQuantity;
	struct AlgoQuantity;

	Vicks();

	void onReset() override {
		for (int i = 0; i < NUM_ALGOS; i++) {
			seed[i] = 0.3f;
			scan[i] = 0.f;
			basis[i] = 0.375f;
			time[i] = 0.65f;
		}
		basisGlobal = 0.375f;
		timeGlobal = 0.65f;
		volIn = 4.f / 7.f;
		volOut = 0.8f;
		globalBasisTime = false;
		calLow = 1.f;
		calHigh = 3.f;
		encMode = ENC_BASIS_TIME;
		volMode = false;
		uiMode = UI_NORMAL;
		calSaw = false;
	}

	void onRandomize(const RandomizeEvent& e) override {
		Module::onRandomize(e);
		randomizeSeedScan();
	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		chorus.init(e.sampleRate);
		proc.init(e.sampleRate);
	}

	void randomizeSeedScan() {
		seed[algo] = random::uniform();
		scan[algo] = random::uniform();
	}

	float& target(int t) {
		switch (t) {
			case T_BASIS: return globalBasisTime ? basisGlobal : basis[algo];
			case T_TIME: return globalBasisTime ? timeGlobal : time[algo];
			case T_SEED: return seed[algo];
			case T_SCAN: return scan[algo];
			case T_VOL_IN: return volIn;
			default: return volOut;
		}
	}

	int encoderTarget(int side) const {
		if (volMode)
			return side ? T_VOL_OUT : T_VOL_IN;
		if (encMode == ENC_SEED_SCAN)
			return side ? T_SCAN : T_SEED;
		return side ? T_TIME : T_BASIS;
	}

	static const char* targetName(int t) {
		static const char* names[NUM_TARGETS] = {"BASIS", "TIME", "SEED", "SCAN", "Input volume", "Output volume"};
		return names[t];
	}

	std::string targetDisplay(int t) {
		float v = target(t);
		switch (t) {
			case T_BASIS: {
				float hz = BASIS_LOW_HZ * std::pow(2.f, v * BASIS_OCTAVES);
				return string::f("%.2f Hz (%s)", hz, noteName(hz).c_str());
			}
			case T_TIME:
				return string::f("%.1f ms", timeMs(v));
			case T_VOL_IN:
				return v <= 0.f ? "-inf dB" : string::f("%+.1f dB", volInDb(v));
			case T_VOL_OUT:
				return v <= 0.f ? "-inf dB" : string::f("%+.1f dB", volOutDb(v));
			default:
				return string::f("%.1f%%", v * 100.f);
		}
	}

	// Typed entry: Hz for BASIS, ms for TIME, dB for volumes, % otherwise
	void setTargetFromDisplay(int t, float x) {
		float v;
		switch (t) {
			case T_BASIS: v = std::log2(std::max(x, 1e-3f) / BASIS_LOW_HZ) / BASIS_OCTAVES; break;
			case T_TIME: v = std::log2(std::max(x, 1e-3f) / 2.f) / 10.f; break;
			case T_VOL_IN: v = (x + 40.f) / 70.f; break;
			case T_VOL_OUT: v = (x + 48.f) / 60.f; break;
			default: v = x / 100.f; break;
		}
		target(t) = clamp(v, 0.f, 1.f);
	}

	int algoFromSwitches() {
		// Up = 0, middle = 1, down = 2 on each toggle
		int l = 2 - (int) std::round(params[ALGO_L_PARAM].getValue());
		int r = 2 - (int) std::round(params[ALGO_R_PARAM].getValue());
		return clamp(l * 3 + r, 0, NUM_ALGOS - 1);
	}

	// Button handling: short presses act on release, both together randomise
	// (or confirm), and holding one alone for 5 s enters a setup mode.
	void buttons(float dt) {
		bool t = params[MODE_PARAM].getValue() > 0.5f;
		bool b = params[BANK_PARAM].getValue() > 0.5f;
		if (t && !topDown) {
			topDown = true;
			topTime = 0.f;
			topLong = false;
			if (botDown && !combo) {
				combo = true;
				onBoth();
			}
		}
		if (b && !botDown) {
			botDown = true;
			botTime = 0.f;
			botLong = false;
			if (topDown && !combo) {
				combo = true;
				onBoth();
			}
		}
		if (topDown) {
			topTime += dt;
			if (!combo && !topLong && topTime >= HOLD_TIME) {
				topLong = true;
				onTopLong();
			}
		}
		if (botDown) {
			botTime += dt;
			if (!combo && !botLong && botTime >= HOLD_TIME) {
				botLong = true;
				onBotLong();
			}
		}
		if (!t && topDown) {
			topDown = false;
			if (!combo && !topLong)
				onTopShort();
		}
		if (!b && botDown) {
			botDown = false;
			if (!combo && !botLong)
				onBotShort();
		}
		if (!topDown && !botDown)
			combo = false;
	}

	float basisRawCv() {
		// BASIS jack through its attenuverter, before calibration
		return inputs[BASIS_INPUT].getVoltage() * params[BASIS_ATT_PARAM].getValue();
	}

	void onTopShort() {
		switch (uiMode) {
			case UI_NORMAL:
				volMode = false;
				encMode = (encMode == ENC_BASIS_TIME) ? ENC_SEED_SCAN : ENC_BASIS_TIME;
				break;
			case UI_SAVE_SELECT:
				pendingGlobal = true;
				break;
			case UI_CALIBRATE: {
				float v = basisRawCv();
				if (std::fabs(v - 3.f) < 0.3f)
					calHighPending = v;
				else
					errorTimer = 1.f;
			} break;
		}
	}

	void onBotShort() {
		switch (uiMode) {
			case UI_NORMAL:
				if (encMode == ENC_BASIS_TIME)
					volMode = !volMode;
				else
					bankFlash = 0.4f; // only one bank of algorithms
				break;
			case UI_SAVE_SELECT:
				pendingGlobal = false;
				break;
			case UI_CALIBRATE: {
				float v = basisRawCv();
				if (std::fabs(v - 1.f) < 0.3f)
					calLowPending = v;
				else
					errorTimer = 1.f;
			} break;
		}
	}

	void onBoth() {
		switch (uiMode) {
			case UI_NORMAL:
				randomizeSeedScan();
				break;
			case UI_SAVE_SELECT:
				if (pendingGlobal != globalBasisTime) {
					// Carry the current values over so the sound doesn't jump
					float b = target(T_BASIS), tm = target(T_TIME);
					globalBasisTime = pendingGlobal;
					target(T_BASIS) = b;
					target(T_TIME) = tm;
					gen.reset();
				}
				uiMode = UI_NORMAL;
				break;
			case UI_CALIBRATE:
				calLow = calLowPending;
				calHigh = calHighPending;
				calSaw = false;
				uiMode = UI_NORMAL;
				break;
		}
	}

	void onTopLong() {
		if (uiMode == UI_NORMAL) {
			uiMode = UI_SAVE_SELECT;
			pendingGlobal = globalBasisTime;
		}
		else if (uiMode == UI_CALIBRATE) {
			calSaw = !calSaw;
		}
	}

	void onBotLong() {
		if (uiMode == UI_NORMAL) {
			uiMode = UI_CALIBRATE;
			calLowPending = calLow;
			calHighPending = calHigh;
			volMode = false;
		}
		else if (uiMode == UI_CALIBRATE) {
			calLowPending = 1.f;
			calHighPending = 3.f;
		}
	}

	float knobCv(int param, int input) {
		// 5 V spans a unipolar knob's full range
		return params[param].getValue() + inputs[input].getVoltage() / 5.f;
	}

	float bipolarCv(int param, int input) {
		return params[param].getValue() + inputs[input].getVoltage() * 0.4f;
	}

	void control(float fs, float dt) {
		buttons(dt);

		// Encoders: apply the knob's movement to whichever parameter they target
		for (int side = 0; side < 2; side++) {
			float v = params[ENC_L_PARAM + side].getValue();
			if (!encInit)
				encLast[side] = v;
			float d = v - encLast[side];
			encLast[side] = v;
			// Ignore jumps (absolute knob modes, undo across a mode change): only
			// relative movement edits the target
			if (d != 0.f && std::fabs(d) < 0.5f) {
				float& tg = target(encoderTarget(side));
				tg = clamp(tg + d, 0.f, 1.f);
			}
		}
		encInit = true;

		// Algorithm change: fade the generator out, switch, fade back in
		pendingAlgo = algoFromSwitches();

		// CV normalling: VCA -> the five attenuverted jacks, MORPH -> the other four,
		// else a small offset so the attenuverters work as fine tune controls
		const bool vcaOn = inputs[VCA_INPUT].isConnected();
		const float vcaV = inputs[VCA_INPUT].getVoltage();
		const bool morphOn = inputs[MORPH_INPUT].isConnected();
		const float morphV = inputs[MORPH_INPUT].getVoltage();
		const float norm4 = morphOn ? morphV : (vcaOn ? vcaV : NORMAL_OFFSET);
		const float morphSrc = morphOn ? morphV : (vcaOn ? vcaV : NORMAL_OFFSET);
		auto src = [&](int input) {
			return inputs[input].isConnected() ? inputs[input].getVoltage() : norm4;
		};
		const float morphCv = morphSrc * params[MORPH_ATT_PARAM].getValue();
		const float fieldCv = src(FIELD_INPUT) * params[FIELD_ATT_PARAM].getValue();
		const float timeCv = src(TIME_INPUT) * params[TIME_ATT_PARAM].getValue();
		const float formCv = src(FORM_INPUT) * params[FORM_ATT_PARAM].getValue();
		float basisCv;
		if (inputs[BASIS_INPUT].isConnected()) {
			// Two-point calibration: calLow should read 1 V and calHigh 3 V
			bool cal = uiMode == UI_CALIBRATE;
			float lo = cal ? calLowPending : calLow;
			float hi = cal ? calHighPending : calHigh;
			basisCv = 1.f + (basisRawCv() - lo) * 2.f / std::max(hi - lo, 0.1f);
		}
		else {
			basisCv = norm4 * params[BASIS_ATT_PARAM].getValue();
		}

		// Targets
		float tg[16];
		tg[0] = target(T_BASIS) * BASIS_OCTAVES + basisCv;
		tg[1] = clamp(bipolarCv(WARP_PARAM, WARP_INPUT), -1.f, 1.f);
		tg[2] = clamp(bipolarCv(SPAN_PARAM, SPAN_INPUT), -1.f, 1.f);
		tg[3] = clamp(params[MORPH_PARAM].getValue() + morphCv / 5.f, 0.f, 1.f);
		tg[4] = clamp(bipolarCv(FUSE_PARAM, FUSE_INPUT), -1.f, 1.f);
		tg[5] = clamp(target(T_SCAN) + inputs[SCAN_INPUT].getVoltage() / 5.f, 0.f, 1.f);
		tg[6] = target(T_SEED);
		tg[7] = clamp(params[FIELD_PARAM].getValue() + fieldCv * 0.4f, -1.f, 1.f);
		tg[8] = clamp(target(T_TIME) + timeCv / 5.f, 0.f, 1.f);
		tg[9] = clamp(params[FORM_PARAM].getValue() + formCv / 5.f, 0.f, 1.f);
		tg[10] = clamp(bipolarCv(FEED_PARAM, FEED_INPUT), -1.f, 1.f);
		tg[11] = clamp(knobCv(CELL_PARAM, CELL_INPUT), 0.f, 1.f);
		tg[12] = clamp(bipolarCv(MIX_PARAM, MIX_INPUT) * 0.5f + 0.5f, 0.f, 1.f);
		tg[13] = volGain(volIn, volInDb(volIn));
		tg[14] = volGain(volOut, volOutDb(volOut));
		tg[15] = 0.f;

		const float coef = 1.f - std::exp(-dt / 0.005f);
		const float seedCoef = 1.f - std::exp(-dt / 0.03f);
		for (int i = 0; i < 16; i++) {
			if (!smInit)
				sm[i] = tg[i];
			else
				sm[i] += (tg[i] - sm[i]) * (i == 6 ? seedCoef : coef);
		}
		smInit = true;

		const float fs2 = fs * 2.f;
		gc.algo = algo;
		gc.f0 = clamp(BASIS_LOW_HZ * dsp::exp2_taylor5(clamp(sm[0], -4.f, 14.f)), 1.f, fs2 * 0.2f);
		gc.warp = sm[1];
		gc.span = sm[2];
		gc.morph = sm[3];
		gc.fuse = sm[4];
		gc.scan = sm[5];
		gc.seed = sm[6];
		gen.control(gc, fs2);
		fx = fxFor(gc);

		const float seedPos = gc.seed * 48.f;
		float notchHz[4];
		for (int k = 0; k < 4; k++)
			notchHz[k] = 150.f * std::pow(2.f, scanRand(500u + k, seedPos) * 5.5f);
		filter.control(sm[7], fx, notchHz, fs);

		pc.timeSamples = timeMs(sm[8]) * 0.001f * fs;
		pc.form = sm[9];
		pc.feed = sm[10] * 1.02f;
		pc.shiftHz = 1200.f * std::pow(sm[11], 2.5f);
		pc.audioMod = fx.audioMod;
		pc.diffuse = algoIsDiffuse(algo);
		pc.tapRatio[0] = 1.f;
		for (int k = 1; k < 4; k++)
			pc.tapRatio[k] = 0.15f + 0.8f * scanRand(600u + k, seedPos);

		mixCos = std::cos(sm[12] * float(M_PI) / 2.f);
		mixSin = std::sin(sm[12] * float(M_PI) / 2.f);
		gainIn = sm[13];
		gainOut = sm[14];

		updateUi(dt);
	}

	void updateUi(float dt) {
		blinkClock = wrap01(blinkClock + dt);
		errorTimer = std::max(errorTimer - dt, 0.f);
		bankFlash = std::max(bankFlash - dt, 0.f);
		const bool blinkSlow = blinkClock < 0.5f;
		const bool blinkFast = std::fmod(blinkClock, 0.2f) < 0.1f;

		// Encoder target labels: white = active, red = inactive
		const bool bt = encMode == ENC_BASIS_TIME;
		if (volMode) {
			for (int i = 0; i < 4; i++)
				uiLabel[i] = 0;
			uiLabel[4] = 0;
			uiLabel[5] = 2;
		}
		else {
			uiLabel[0] = uiLabel[1] = bt ? 1 : 2;
			uiLabel[2] = uiLabel[3] = bt ? 2 : 1;
			uiLabel[4] = bt ? 0 : 1;
			uiLabel[5] = bt ? 1 : 0;
		}

		// Light windows
		clipIn = std::max(clipIn - dt * 4.f, 0.f);
		clipOut = std::max(clipOut - dt * 4.f, 0.f);
		if (volMode) {
			uiWindow[0] = 1.f;
			uiWindowGlow[0] = clipIn;
			uiWindow[1] = 1.f;
			uiWindowGlow[1] = clipOut;
			uiWindow[2] = volIn;
			uiWindow[3] = volOut;
			uiWindowGlow[2] = uiWindowGlow[3] = 1.f;
		}
		else {
			uiWindow[0] = target(T_SEED);
			uiWindow[1] = target(T_SCAN);
			uiWindow[2] = target(T_BASIS);
			uiWindow[3] = target(T_TIME);
			for (int i = 0; i < 4; i++)
				uiWindowGlow[i] = 0.9f;
		}
		if (uiMode == UI_SAVE_SELECT) {
			// Top windows = global, bottom windows = per algorithm
			for (int i = 0; i < 4; i++) {
				bool sel = (i < 2) == pendingGlobal;
				uiWindow[i] = 1.f;
				uiWindowGlow[i] = sel ? 1.f : 0.08f;
			}
		}
		if (uiMode == UI_CALIBRATE)
			uiWindowGlow[2] = blinkFast ? 1.f : 0.f;

		// Bank lights: bank 1 lights the middle LED
		for (int i = 0; i < 3; i++) {
			float b = (i == 1) ? 1.f : 0.f;
			if (bankFlash > 0.f)
				b = blinkFast ? 1.f : 0.f;
			if (errorTimer > 0.f)
				b = blinkFast ? 1.f : 0.f;
			lights[BANK_LIGHT + i].setBrightness(b);
		}
		lights[VOL_LIGHT].setBrightness(volMode ? 1.f : 0.f);

		float logoL = 0.f, logoR = 0.f;
		if (uiMode == UI_SAVE_SELECT) {
			logoL = logoR = blinkSlow ? 1.f : 0.f;
		}
		else if (uiMode == UI_CALIBRATE) {
			float v = basisRawCv();
			logoL = std::fabs(v - 1.f) < 0.3f ? 1.f : 0.f;
			logoR = std::fabs(v - 3.f) < 0.3f ? 1.f : 0.f;
		}
		lights[LOGO_LIGHT + 0].setBrightness(logoL);
		lights[LOGO_LIGHT + 1].setBrightness(logoR);

		// Level meters: 0 dB at the top of the bar is 10 V in, 9 V out
		const float full[4] = {10.f, 10.f, 9.f, 9.f};
		for (int i = 0; i < 4; i++) {
			float db = 20.f * std::log10(std::max(peak[i] / full[i], 1e-5f));
			uiMeter[i] = clamp(1.f + db / 42.f, 0.f, 1.f);
			peak[i] *= std::exp(-dt / 0.15f);
		}
	}

	void process(const ProcessArgs& args) override {
		const float fs = args.sampleRate;
		if (controlCounter-- <= 0) {
			controlCounter = CONTROL_RATE - 1;
			control(fs, args.sampleTime * CONTROL_RATE);
		}

		// Algorithm switching with a 4 ms fade
		const float fadeStep = 1.f / (0.004f * fs);
		if (pendingAlgo != algo) {
			fade -= fadeStep;
			if (fade <= 0.f) {
				fade = 0.f;
				algo = pendingAlgo;
				gen.reset();
				controlCounter = 0;
			}
		}
		else {
			fade = std::min(fade + fadeStep, 1.f);
		}

		// Generator, 2x oversampled
		float bufL[2], bufR[2];
		for (int j = 0; j < 2; j++)
			gen.process(&bufL[j], &bufR[j]);
		float genL = decim[0].process(bufL) * fade;
		float genR = decim[1].process(bufR) * fade;

		// External stereo input (L and R normalled together)
		const bool inL = inputs[IN_L_INPUT].isConnected();
		const bool inR = inputs[IN_R_INPUT].isConnected();
		float exL = inL ? inputs[IN_L_INPUT].getVoltage() : inputs[IN_R_INPUT].getVoltage();
		float exR = inR ? inputs[IN_R_INPUT].getVoltage() : exL;
		exL *= gainIn * 0.2f;
		exR *= gainIn * 0.2f;
		peak[0] = std::max(peak[0], std::fabs(exL) * 5.f);
		peak[1] = std::max(peak[1], std::fabs(exR) * 5.f);
		if (std::fabs(exL) > 1.6f || std::fabs(exR) > 1.6f)
			clipIn = 1.f;
		exL = softClip(exL, 2.f);
		exR = softClip(exR, 2.f);

		float l = genL + exL;
		float r = genR + exR;
		chorus.process(&l, &r, fx.chorus, fs);

		// Filter section, then the parallel sine (one algorithm) and the VCA
		l = filter.process(0, l);
		r = filter.process(1, r);
		const float sine = sin2pi(sinePhase) * 0.5f * fx.sineLevel * fade;
		sinePhase = wrap01(sinePhase + gc.f0 / fs);
		l += sine;
		r += sine;
		const float vca = inputs[VCA_INPUT].isConnected() ? clamp(inputs[VCA_INPUT].getVoltage() / 5.f, 0.f, 2.4f) : 1.f;
		l *= vca;
		r *= vca;

		// Processor
		float wetL, wetR;
		proc.process(softClip(l, 2.f), softClip(r, 2.f), 0.5f * (genL + genR), pc, &wetL, &wetR);

		float outL = (l * mixCos + wetL * mixSin) * gainOut;
		float outR = (r * mixCos + wetR * mixSin) * gainOut;
		if (calSaw) {
			// Calibration reference tone replaces the algorithm
			float d = gc.f0 / fs;
			float saw = 2.f * sawPhase - 1.f - polyBlep(sawPhase, d);
			sawPhase = wrap01(sawPhase + d);
			outL = outR = saw * 0.5f * gainOut;
		}

		// 18 Vpp full scale, soft clipped
		if (std::fabs(outL) > 1.6f || std::fabs(outR) > 1.6f)
			clipOut = 1.f;
		float vL = 9.f * fastTanh(outL * 5.f / 9.f);
		float vR = 9.f * fastTanh(outR * 5.f / 9.f);
		peak[2] = std::max(peak[2], std::fabs(vL));
		peak[3] = std::max(peak[3], std::fabs(vR));

		// L is normalled into R: with only R patched it carries the mono sum
		if (outputs[OUT_L_OUTPUT].isConnected()) {
			outputs[OUT_L_OUTPUT].setVoltage(vL);
			outputs[OUT_R_OUTPUT].setVoltage(vR);
		}
		else {
			outputs[OUT_R_OUTPUT].setVoltage(0.5f * (vL + vR));
		}
	}

	json_t* dataToJson() override {
		json_t* root = json_object();
		auto arr = [](const float* v) {
			json_t* a = json_array();
			for (int i = 0; i < NUM_ALGOS; i++)
				json_array_append_new(a, json_real(v[i]));
			return a;
		};
		json_object_set_new(root, "seed", arr(seed));
		json_object_set_new(root, "scan", arr(scan));
		json_object_set_new(root, "basis", arr(basis));
		json_object_set_new(root, "time", arr(time));
		json_object_set_new(root, "basisGlobal", json_real(basisGlobal));
		json_object_set_new(root, "timeGlobal", json_real(timeGlobal));
		json_object_set_new(root, "volIn", json_real(volIn));
		json_object_set_new(root, "volOut", json_real(volOut));
		json_object_set_new(root, "globalBasisTime", json_boolean(globalBasisTime));
		json_object_set_new(root, "calLow", json_real(calLow));
		json_object_set_new(root, "calHigh", json_real(calHigh));
		json_object_set_new(root, "encMode", json_integer(encMode));
		return root;
	}

	void dataFromJson(json_t* root) override {
		auto arr = [&](const char* key, float* v) {
			json_t* a = json_object_get(root, key);
			if (!a)
				return;
			for (int i = 0; i < NUM_ALGOS && i < (int) json_array_size(a); i++)
				v[i] = clamp((float) json_number_value(json_array_get(a, i)), 0.f, 1.f);
		};
		auto num = [&](const char* key, float* v) {
			json_t* j = json_object_get(root, key);
			if (j)
				*v = (float) json_number_value(j);
		};
		arr("seed", seed);
		arr("scan", scan);
		arr("basis", basis);
		arr("time", time);
		num("basisGlobal", &basisGlobal);
		num("timeGlobal", &timeGlobal);
		num("volIn", &volIn);
		num("volOut", &volOut);
		num("calLow", &calLow);
		num("calHigh", &calHigh);
		if (json_t* j = json_object_get(root, "globalBasisTime"))
			globalBasisTime = json_boolean_value(j);
		if (json_t* j = json_object_get(root, "encMode"))
			encMode = json_integer_value(j) ? ENC_SEED_SCAN : ENC_BASIS_TIME;
		encInit = false;
	}
};


// Endless encoder: the param only counts movement; its tooltip and typed
// value show and edit whichever parameter the encoder currently targets.
struct Vicks::EncoderQuantity : ParamQuantity {
	int side = 0;
	Vicks* vicks() {
		return dynamic_cast<Vicks*>(module);
	}
	std::string getLabel() override {
		Vicks* m = vicks();
		if (!m)
			return name;
		return name + " (" + targetName(m->encoderTarget(side)) + ")";
	}
	std::string getDisplayValueString() override {
		Vicks* m = vicks();
		return m ? m->targetDisplay(m->encoderTarget(side)) : "";
	}
	void setDisplayValueString(std::string s) override {
		Vicks* m = vicks();
		if (m && !s.empty())
			m->setTargetFromDisplay(m->encoderTarget(side), (float) std::atof(s.c_str()));
	}
	std::string getUnit() override {
		return "";
	}
};

struct Vicks::AlgoQuantity : SwitchQuantity {
	std::string getDisplayValueString() override {
		Vicks* m = dynamic_cast<Vicks*>(module);
		std::string s = SwitchQuantity::getDisplayValueString();
		if (m) {
			int a = m->algoFromSwitches();
			s += string::f(" (algorithm %d: %s)", a + 1, algoName(a));
		}
		return s;
	}
};


Vicks::Vicks() {
	config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

	configParam(WARP_PARAM, -1.f, 1.f, 0.f, "WARP (cross-modulation)", "%", 0.f, 100.f);
	configParam(SPAN_PARAM, -1.f, 1.f, 0.f, "SPAN (pitch / stereo spread)", "%", 0.f, 100.f);
	configParam(MIX_PARAM, -1.f, 1.f, 0.f, "MIX (generator / processor)", "%", 0.f, 100.f);
	configParam(FEED_PARAM, -1.f, 1.f, 0.f, "FEED (processor feedback)", "%", 0.f, 100.f);
	configParam(MORPH_PARAM, 0.f, 1.f, 0.5f, "MORPH (timbre / cluster scan)", "%", 0.f, 100.f);
	configParam(FORM_PARAM, 0.f, 1.f, 0.3f, "FORM (processor structure)", "%", 0.f, 100.f);
	configParam(FUSE_PARAM, -1.f, 1.f, 0.f, "FUSE (timbral effect)", "%", 0.f, 100.f);
	configParam(FIELD_PARAM, -1.f, 1.f, 0.f, "FIELD (low pass / high pass)", "%", 0.f, 100.f);
	configParam(CELL_PARAM, 0.f, 1.f, 0.f, "CELL (frequency shift)", "%", 0.f, 100.f);
	configParam(MORPH_ATT_PARAM, -1.f, 1.f, 0.f, "MORPH CV attenuverter", "%", 0.f, 100.f);
	configParam(BASIS_ATT_PARAM, -1.f, 1.f, 0.f, "BASIS CV attenuverter (1V/oct at max)", "%", 0.f, 100.f);
	configParam(FIELD_ATT_PARAM, -1.f, 1.f, 0.f, "FIELD CV attenuverter", "%", 0.f, 100.f);
	configParam(TIME_ATT_PARAM, -1.f, 1.f, 0.f, "TIME CV attenuverter", "%", 0.f, 100.f);
	configParam(FORM_ATT_PARAM, -1.f, 1.f, 0.f, "FORM CV attenuverter", "%", 0.f, 100.f);

	for (int side = 0; side < 2; side++) {
		auto* q = configParam<EncoderQuantity>(ENC_L_PARAM + side, -1000.f, 1000.f, 0.f, side ? "Right encoder" : "Left encoder");
		q->side = side;
		q->resetEnabled = false;
		q->randomizeEnabled = false;
	}

	const std::vector<std::string> pos = {"Down", "Middle", "Up"};
	configSwitch<AlgoQuantity>(ALGO_L_PARAM, 0.f, 2.f, 2.f, "Algorithm row", pos);
	configSwitch<AlgoQuantity>(ALGO_R_PARAM, 0.f, 2.f, 2.f, "Algorithm column", pos);
	configButton(MODE_PARAM, "MODE (BASIS + TIME / SEED + SCAN)");
	configButton(BANK_PARAM, "BANK / VOL");

	configInput(WARP_INPUT, "WARP CV");
	configInput(SPAN_INPUT, "SPAN CV");
	configInput(FUSE_INPUT, "FUSE CV");
	configInput(IN_L_INPUT, "Audio left");
	configInput(IN_R_INPUT, "Audio right");
	configInput(VCA_INPUT, "VCA (0-5V, normalled to the five attenuverted CV inputs)");
	configInput(MORPH_INPUT, "MORPH CV (normalled to BASIS, FIELD, TIME, FORM)");
	configInput(BASIS_INPUT, "BASIS CV (1V/oct)");
	configInput(FIELD_INPUT, "FIELD CV");
	configInput(TIME_INPUT, "TIME CV");
	configInput(FORM_INPUT, "FORM CV");
	configInput(CELL_INPUT, "CELL CV");
	configInput(MIX_INPUT, "MIX CV");
	configInput(FEED_INPUT, "FEED CV");
	configInput(SCAN_INPUT, "SCAN CV");
	configOutput(OUT_L_OUTPUT, "Left");
	configOutput(OUT_R_OUTPUT, "Right (mono sum when left is unpatched)");

	configLight(VOL_LIGHT, "Volume mode");
	configLight(LOGO_LIGHT + 0, "Calibration low point");
	configLight(LOGO_LIGHT + 1, "Calibration high point");

	onReset();
	gen.reset();
	float fs = (APP && APP->engine) ? APP->engine->getSampleRate() : 48000.f;
	chorus.init(fs);
	proc.init(fs);
	// Build the shared wavetables now rather than on the audio thread
	WaveBank::get();
}


// ---------------------------------------------------------------------------
// Panel

namespace {

NVGcolor cream() {
	return nvgRGB(0xe4, 0xd8, 0xbc);
}
NVGcolor line() {
	return nvgRGBA(0xc9, 0xb3, 0x86, 0xb0);
}
NVGcolor glowRed() {
	return nvgRGB(0xff, 0x32, 0x1e);
}

std::shared_ptr<window::Font> panelFont() {
	return APP->window->loadFont(asset::system("res/fonts/DejaVuSans.ttf"));
}

void text(NVGcontext* vg, std::shared_ptr<window::Font> font, Vec p, const char* s, float size, NVGcolor color, int align = NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, float spacing = 0.6f) {
	if (!font)
		return;
	nvgFontFaceId(vg, font->handle);
	nvgFontSize(vg, size);
	nvgTextLetterSpacing(vg, spacing);
	nvgFillColor(vg, color);
	nvgTextAlign(vg, align);
	nvgText(vg, p.x, p.y, s, NULL);
}

} // namespace


// Static artwork, drawn once into a framebuffer. Coordinates in mm.
struct PanelDecor : TransparentWidget {
	struct Label {
		Vec pos;
		std::string text;
		float size;
		bool boxed;
	};
	struct Box {
		math::Rect rect;
		NVGcolor fill;
	};
	std::vector<Label> labels;
	std::vector<Box> boxes;
	std::vector<std::vector<Vec>> dashed;
	std::vector<std::pair<Vec, Vec>> arrows;

	void draw(const DrawArgs& args) override {
		NVGcontext* vg = args.vg;
		auto font = panelFont();

		// Outer frame line
		nvgBeginPath(vg);
		nvgRoundedRect(vg, mm2px(1.2f), mm2px(6.2f), box.size.x - mm2px(2.4f), box.size.y - mm2px(12.4f), 6.f);
		nvgStrokeColor(vg, nvgRGBA(0xc9, 0xb3, 0x86, 0x40));
		nvgStrokeWidth(vg, 0.8f);
		nvgStroke(vg);

		for (const Box& b : boxes) {
			nvgBeginPath(vg);
			nvgRoundedRect(vg, mm2px(b.rect.pos.x), mm2px(b.rect.pos.y), mm2px(b.rect.size.x), mm2px(b.rect.size.y), 3.f);
			nvgFillColor(vg, b.fill);
			nvgFill(vg);
			nvgStrokeColor(vg, line());
			nvgStrokeWidth(vg, 0.8f);
			nvgStroke(vg);
		}

		// Dashed CV routing from the attenuverters to their jacks
		nvgStrokeColor(vg, nvgRGBA(0xc9, 0xb3, 0x86, 0x90));
		nvgStrokeWidth(vg, 0.7f);
		for (const auto& path : dashed) {
			for (size_t i = 0; i + 1 < path.size(); i++) {
				Vec a = mm2px(path[i]), b = mm2px(path[i + 1]);
				float len = a.minus(b).norm();
				int n = std::max(1, (int) (len / 4.f));
				for (int k = 0; k < n; k++) {
					Vec p0 = a.plus(b.minus(a).mult((float) k / n));
					Vec p1 = a.plus(b.minus(a).mult((k + 0.55f) / n));
					nvgBeginPath(vg);
					nvgMoveTo(vg, p0.x, p0.y);
					nvgLineTo(vg, p1.x, p1.y);
					nvgStroke(vg);
				}
			}
		}

		// Small arrows (normalling indicators)
		nvgFillColor(vg, cream());
		for (const auto& a : arrows) {
			Vec p = mm2px(a.first);
			Vec d = a.second;
			Vec n(-d.y, d.x);
			nvgBeginPath(vg);
			nvgMoveTo(vg, p.x + d.x * 3.f, p.y + d.y * 3.f);
			nvgLineTo(vg, p.x - d.x * 2.f + n.x * 2.5f, p.y - d.y * 2.f + n.y * 2.5f);
			nvgLineTo(vg, p.x - d.x * 2.f - n.x * 2.5f, p.y - d.y * 2.f - n.y * 2.5f);
			nvgClosePath(vg);
			nvgFill(vg);
		}

		// Title
		{
			Vec c = mm2px(Vec(61.f, 3.6f));
			nvgBeginPath(vg);
			nvgRect(vg, c.x - mm2px(9.f), c.y - mm2px(2.2f), mm2px(18.f), mm2px(4.4f));
			nvgFillColor(vg, nvgRGB(0x15, 0x14, 0x17));
			nvgFill(vg);
			nvgStrokeColor(vg, line());
			nvgStrokeWidth(vg, 0.8f);
			nvgStroke(vg);
			text(vg, font, c, "VICKS", 11.f, cream(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 3.f);
			text(vg, font, mm2px(Vec(61.f, 125.3f)), "STEREO DRONE VOICE", 6.f, nvgRGBA(0xe4, 0xd8, 0xbc, 0xa0), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 1.5f);
		}

		for (const Label& l : labels) {
			Vec p = mm2px(l.pos);
			if (l.boxed && font) {
				nvgFontFaceId(vg, font->handle);
				nvgFontSize(vg, l.size);
				nvgTextLetterSpacing(vg, 0.8f);
				float bounds[4];
				nvgTextBounds(vg, 0, 0, l.text.c_str(), NULL, bounds);
				float w = bounds[2] - bounds[0] + 8.f;
				float h = l.size + 4.f;
				nvgBeginPath(vg);
				nvgRect(vg, p.x - w / 2.f, p.y - h / 2.f, w, h);
				nvgStrokeColor(vg, line());
				nvgStrokeWidth(vg, 0.8f);
				nvgStroke(vg);
			}
			text(vg, font, p, l.text.c_str(), l.size, cream(), NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
		}
	}
};


// Labels that light up: white for the active encoder target, red when inactive
struct ModeLabel : TransparentWidget {
	Vicks* module = nullptr;
	int index = 0;
	std::string label;
	bool boxed = false;

	int state() {
		if (!module) {
			static const int preview[6] = {1, 1, 2, 2, 0, 1};
			return preview[index];
		}
		return module->uiLabel[index];
	}
	void drawText(const DrawArgs& args, NVGcolor color) {
		Vec c = box.size.div(2.f);
		if (boxed) {
			nvgBeginPath(args.vg);
			nvgRect(args.vg, 0.5f, 0.5f, box.size.x - 1.f, box.size.y - 1.f);
			nvgStrokeColor(args.vg, line());
			nvgStrokeWidth(args.vg, 0.8f);
			nvgStroke(args.vg);
		}
		text(args.vg, panelFont(), c, label.c_str(), 8.f, color, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE, 0.8f);
	}
	void draw(const DrawArgs& args) override {
		if (state() == 0)
			drawText(args, nvgRGBA(0x80, 0x78, 0x6a, 0xff));
	}
	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer != 1)
			return;
		int s = state();
		if (s == 1)
			drawText(args, glowRed());
		else if (s == 2)
			drawText(args, nvgRGB(0xff, 0xfb, 0xf0));
	}
};


// Rectangular light window with a horizontal bar showing a value
struct ValueWindow : TransparentWidget {
	Vicks* module = nullptr;
	int index = 0;

	void draw(const DrawArgs& args) override {
		nvgBeginPath(args.vg);
		nvgRect(args.vg, 0, 0, box.size.x, box.size.y);
		nvgFillColor(args.vg, nvgRGB(0x2a, 0x0b, 0x09));
		nvgFill(args.vg);
		nvgStrokeColor(args.vg, line());
		nvgStrokeWidth(args.vg, 0.7f);
		nvgStroke(args.vg);
	}
	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer != 1)
			return;
		float fill = module ? module->uiWindow[index] : 0.5f;
		float glow = module ? module->uiWindowGlow[index] : 0.9f;
		if (glow <= 0.01f || fill <= 0.f)
			return;
		const float pad = 1.2f;
		const float w = (box.size.x - 2.f * pad) * clamp(fill, 0.f, 1.f);
		// Horizontal LED lines, like a segmented window
		const int rows = 4;
		const float rh = (box.size.y - 2.f * pad) / rows;
		for (int i = 0; i < rows; i++) {
			nvgBeginPath(args.vg);
			nvgRect(args.vg, pad, pad + i * rh, w, rh * 0.6f);
			nvgFillColor(args.vg, nvgRGBAf(1.f, 0.2f, 0.12f, glow));
			nvgFill(args.vg);
		}
	}
};


// Level meter strip: white for left, red for right
struct LevelMeter : TransparentWidget {
	Vicks* module = nullptr;
	int first = 0;

	void draw(const DrawArgs& args) override {
		nvgBeginPath(args.vg);
		nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, box.size.y / 2.f);
		nvgFillColor(args.vg, nvgRGB(0x1e, 0x1b, 0x1a));
		nvgFill(args.vg);
		nvgStrokeColor(args.vg, line());
		nvgStrokeWidth(args.vg, 0.7f);
		nvgStroke(args.vg);
	}
	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer != 1 || !module)
			return;
		const float pad = 1.5f;
		const float h = (box.size.y - 2.f * pad) / 2.f;
		for (int c = 0; c < 2; c++) {
			float v = module->uiMeter[first + c];
			if (v <= 0.f)
				continue;
			nvgBeginPath(args.vg);
			nvgRect(args.vg, pad, pad + c * h, (box.size.x - 2.f * pad) * v, h * 0.85f);
			nvgFillColor(args.vg, c ? nvgRGB(0xff, 0x30, 0x20) : nvgRGB(0xf4, 0xf0, 0xe6));
			nvgFill(args.vg);
		}
	}
};


// Rectangular red LED for the bank indicator
struct BankLed : RectangleLight<RedLight> {
	BankLed() {
		box.size = mm2px(Vec(3.4f, 1.5f));
		bgColor = nvgRGB(0x2a, 0x0b, 0x09);
		borderColor = nvgRGBA(0xc9, 0xb3, 0x86, 0x80);
	}
};


// Small emblem with a red LED behind it (used during calibration / setup)
struct LogoLight : SmallLight<RedLight> {
	void drawBackground(const DrawArgs& args) override {
		SmallLight<RedLight>::drawBackground(args);
		Vec c = box.size.div(2.f);
		nvgStrokeColor(args.vg, nvgRGBA(0xc9, 0x40, 0x30, 0xc0));
		nvgStrokeWidth(args.vg, 0.8f);
		for (int i = 0; i < 4; i++) {
			float a = i * float(M_PI) / 2.f;
			nvgBeginPath(args.vg);
			nvgEllipse(args.vg, c.x + std::cos(a) * 4.f, c.y + std::sin(a) * 4.f, 4.f, 4.f);
			nvgStroke(args.vg);
		}
		nvgBeginPath(args.vg);
		nvgRect(args.vg, c.x - 9.f, c.y - 9.f, 18.f, 18.f);
		nvgStroke(args.vg);
	}
};


// Endless encoder with a knurled cap (no pointer, like a hardware encoder)
struct EncoderKnob : app::Knob {
	EncoderKnob() {
		box.size = mm2px(Vec(17.f, 17.f));
		smooth = false;
	}
	// Own drag handling: Rack scales knob drags by the angle span, which suits
	// bounded knobs but not an endless encoder. ~350 px of drag covers a
	// parameter's range; Ctrl drags 10x finer. Knob's drag start/end still
	// record undo history.
	void onDragMove(const DragMoveEvent& e) override {
		if (e.button != GLFW_MOUSE_BUTTON_LEFT)
			return;
		ParamQuantity* pq = getParamQuantity();
		if (!pq)
			return;
		float delta = (horizontal ? e.mouseDelta.x : -e.mouseDelta.y) / 350.f;
		if (APP->window->getMods() & RACK_MOD_CTRL)
			delta *= 0.1f;
		pq->setValue(pq->getValue() + delta);
	}
	void onHoverScroll(const HoverScrollEvent& e) override {
		ParamQuantity* pq = getParamQuantity();
		if (!pq)
			return;
		float step = (e.scrollDelta.y > 0.f) ? 0.01f : -0.01f;
		if (APP->window->getMods() & RACK_MOD_CTRL)
			step *= 0.1f;
		pq->setValue(pq->getValue() + step);
		e.consume(this);
	}
	void draw(const DrawArgs& args) override {
		NVGcontext* vg = args.vg;
		ParamQuantity* pq = getParamQuantity();
		float angle = pq ? pq->getValue() * 3.f * float(M_PI) : 0.f;
		Vec c = box.size.div(2.f);
		float r = box.size.x / 2.f;

		// Shadow
		nvgBeginPath(vg);
		nvgCircle(vg, c.x, c.y + 2.f, r);
		nvgFillColor(vg, nvgRGBA(0, 0, 0, 0x60));
		nvgFill(vg);
		// Skirt
		nvgBeginPath(vg);
		nvgCircle(vg, c.x, c.y, r);
		nvgFillPaint(vg, nvgLinearGradient(vg, 0, 0, 0, box.size.y, nvgRGB(0x3a, 0x3a, 0x3c), nvgRGB(0x0c, 0x0c, 0x0d)));
		nvgFill(vg);
		// Knurling
		const int ridges = 36;
		nvgStrokeColor(vg, nvgRGBA(0x00, 0x00, 0x00, 0xb0));
		nvgStrokeWidth(vg, 1.2f);
		for (int i = 0; i < ridges; i++) {
			float a = angle + i * 2.f * float(M_PI) / ridges;
			nvgBeginPath(vg);
			nvgMoveTo(vg, c.x + std::cos(a) * r * 0.80f, c.y + std::sin(a) * r * 0.80f);
			nvgLineTo(vg, c.x + std::cos(a) * r * 0.98f, c.y + std::sin(a) * r * 0.98f);
			nvgStroke(vg);
		}
		// Cap
		nvgBeginPath(vg);
		nvgCircle(vg, c.x, c.y, r * 0.78f);
		nvgFillPaint(vg, nvgRadialGradient(vg, c.x - r * 0.2f, c.y - r * 0.3f, r * 0.1f, r * 0.9f, nvgRGB(0x34, 0x34, 0x36), nvgRGB(0x16, 0x16, 0x17)));
		nvgFill(vg);
		nvgStrokeColor(vg, nvgRGBA(0x60, 0x60, 0x60, 0x80));
		nvgStrokeWidth(vg, 0.8f);
		nvgStroke(vg);
		// Faint texture dot so rotation is visible
		Vec d = c.plus(Vec(std::cos(angle - float(M_PI) / 2.f), std::sin(angle - float(M_PI) / 2.f)).mult(r * 0.62f));
		nvgBeginPath(vg);
		nvgCircle(vg, d.x, d.y, 1.2f);
		nvgFillColor(vg, nvgRGBA(0x70, 0x70, 0x70, 0x90));
		nvgFill(vg);
	}
};


// Slider in the context menu for an encoder target
struct TargetQuantity : Quantity {
	Vicks* module;
	int t;
	TargetQuantity(Vicks* m, int t) : module(m), t(t) {}
	void setValue(float v) override {
		module->target(t) = clamp(v, 0.f, 1.f);
	}
	float getValue() override {
		return module->target(t);
	}
	float getDefaultValue() override {
		static const float defs[Vicks::NUM_TARGETS] = {0.375f, 0.65f, 0.3f, 0.f, 4.f / 7.f, 0.8f};
		return defs[t];
	}
	std::string getLabel() override {
		return Vicks::targetName(t);
	}
	std::string getDisplayValueString() override {
		return module->targetDisplay(t);
	}
	void setDisplayValueString(std::string s) override {
		module->setTargetFromDisplay(t, (float) std::atof(s.c_str()));
	}
};

struct TargetSlider : ui::Slider {
	TargetSlider(Vicks* m, int t) {
		quantity = new TargetQuantity(m, t);
		box.size.x = 220.f;
	}
	~TargetSlider() {
		delete quantity;
	}
};


struct VicksWidget : ModuleWidget {
	PanelDecor* decor;

	VicksWidget(Vicks* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Vicks.svg")));

		auto* fb = new widget::FramebufferWidget;
		fb->box.size = box.size;
		decor = new PanelDecor;
		decor->box.size = box.size;
		fb->addChild(decor);
		addChild(fb);

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		using V = Vicks;

		// --- Jack group boxes (drawn first, underneath everything)
		const NVGcolor inFill = nvgRGB(0x1d, 0x1c, 0x1f);
		const NVGcolor outFill = nvgRGB(0x33, 0x30, 0x2c);
		auto boxAt = [&](float x0, float y0, float x1, float y1, NVGcolor fill) {
			decor->boxes.push_back({math::Rect(Vec(x0, y0), Vec(x1 - x0, y1 - y0)), fill});
		};
		boxAt(46.0f, 6.4f, 76.0f, 21.2f, inFill);    // algorithm selection
		boxAt(1.8f, 88.9f, 21.2f, 102.4f, inFill);   // WARP SPAN
		boxAt(21.6f, 88.9f, 31.0f, 102.4f, inFill);  // FUSE
		boxAt(1.8f, 102.8f, 21.2f, 121.4f, inFill);  // audio in
		boxAt(21.6f, 102.8f, 31.0f, 121.4f, inFill); // VCA
		boxAt(35.6f, 107.6f, 86.4f, 121.4f, inFill); // attenuverted CV
		boxAt(90.9f, 88.9f, 100.3f, 102.4f, inFill); // CELL
		boxAt(100.7f, 88.9f, 120.1f, 102.4f, inFill);// MIX FEED
		boxAt(90.9f, 102.8f, 100.3f, 121.4f, inFill);// SCAN
		boxAt(100.7f, 102.8f, 120.1f, 121.4f, outFill); // outputs

		// --- Top row: generator knobs left, processor knobs right
		knob<RoundLargeBlackKnob>(Vec(11.8f, 16.f), V::WARP_PARAM, "WARP", 9.4f);
		knob<RoundLargeBlackKnob>(Vec(34.4f, 16.f), V::SPAN_PARAM, "SPAN", 9.4f);
		knob<RoundLargeBlackKnob>(Vec(87.5f, 16.f), V::MIX_PARAM, "MIX", 9.4f);
		knob<RoundLargeBlackKnob>(Vec(110.1f, 16.f), V::FEED_PARAM, "FEED", 9.4f);
		addChild(createLightCentered<LogoLight>(mm2px(Vec(23.1f, 10.f)), module, V::LOGO_LIGHT + 0));
		addChild(createLightCentered<LogoLight>(mm2px(Vec(98.8f, 10.f)), module, V::LOGO_LIGHT + 1));

		// Algorithm toggles with the bank LEDs between them
		addParam(createParamCentered<NKK>(mm2px(Vec(52.3f, 13.8f)), module, V::ALGO_L_PARAM));
		addParam(createParamCentered<NKK>(mm2px(Vec(69.7f, 13.8f)), module, V::ALGO_R_PARAM));
		for (int i = 0; i < 3; i++)
			addChild(createLightCentered<BankLed>(mm2px(Vec(61.f, 10.2f + 3.6f * i)), module, V::BANK_LIGHT + i));

		// SEED / MODE / SCAN
		addWindow(Vec(51.f, 24.5f), 0);
		addWindow(Vec(71.f, 24.5f), 1);
		modeLabel(Vec(51.f, 28.6f), 0, "SEED", false);
		modeLabel(Vec(71.f, 28.6f), 1, "SCAN", false);
		decor->labels.push_back({Vec(61.f, 23.4f), "MODE", 6.5f, false});
		addParam(createParamCentered<TL1105>(mm2px(Vec(61.f, 28.f)), module, V::MODE_PARAM));

		// --- Middle: MORPH, encoders, BANK/VOL, FORM
		knob<RoundBigBlackKnob>(Vec(15.9f, 42.5f), V::MORPH_PARAM, "MORPH", 10.8f, true);
		knob<RoundBigBlackKnob>(Vec(106.f, 42.5f), V::FORM_PARAM, "FORM", 10.8f, true);
		addParam(createParamCentered<EncoderKnob>(mm2px(Vec(43.6f, 41.f)), module, V::ENC_L_PARAM));
		addParam(createParamCentered<EncoderKnob>(mm2px(Vec(78.4f, 41.f)), module, V::ENC_R_PARAM));
		modeLabel(Vec(61.f, 35.6f), 4, "BANK", false);
		addParam(createParamCentered<TL1105>(mm2px(Vec(61.f, 41.f)), module, V::BANK_PARAM));
		addChild(createLightCentered<TinyLight<RedLight>>(mm2px(Vec(64.6f, 38.6f)), module, V::VOL_LIGHT));
		modeLabel(Vec(61.f, 46.4f), 5, "VOL", false);
		modeLabel(Vec(43.6f, 52.6f), 2, "BASIS", true);
		modeLabel(Vec(78.4f, 52.6f), 3, "TIME", true);
		addWindow(Vec(43.6f, 56.6f), 2);
		addWindow(Vec(78.4f, 56.6f), 3);

		// --- Lower: FUSE, attenuverters, FIELD, CELL
		knob<RoundLargeBlackKnob>(Vec(11.8f, 70.5f), V::FUSE_PARAM, "FUSE", 9.4f);
		knob<RoundBigBlackKnob>(Vec(61.f, 66.5f), V::FIELD_PARAM, "FIELD", 10.8f, true);
		knob<RoundLargeBlackKnob>(Vec(110.1f, 70.5f), V::CELL_PARAM, "CELL", 9.4f);
		atten(Vec(30.5f, 66.5f), V::MORPH_ATT_PARAM);
		atten(Vec(91.4f, 66.5f), V::FORM_ATT_PARAM);
		atten(Vec(43.6f, 88.f), V::BASIS_ATT_PARAM);
		atten(Vec(61.f, 88.f), V::FIELD_ATT_PARAM);
		atten(Vec(78.4f, 88.f), V::TIME_ATT_PARAM);

		// Dashed routes from the attenuverters to their jacks
		decor->dashed.push_back({Vec(30.5f, 71.f), Vec(33.5f, 74.f), Vec(33.5f, 104.5f), Vec(41.f, 104.5f), Vec(41.f, 108.2f)});
		decor->dashed.push_back({Vec(43.6f, 92.4f), Vec(43.6f, 99.5f), Vec(51.f, 99.5f), Vec(51.f, 108.2f)});
		decor->dashed.push_back({Vec(61.f, 92.4f), Vec(61.f, 108.2f)});
		decor->dashed.push_back({Vec(78.4f, 92.4f), Vec(78.4f, 99.5f), Vec(71.f, 99.5f), Vec(71.f, 108.2f)});
		decor->dashed.push_back({Vec(91.4f, 71.f), Vec(88.4f, 74.f), Vec(88.4f, 104.5f), Vec(81.f, 104.5f), Vec(81.f, 108.2f)});
		// VCA feeds the attenuverted CV block
		decor->dashed.push_back({Vec(31.f, 105.5f), Vec(35.6f, 105.5f), Vec(35.6f, 107.6f)});

		// --- Jacks
		const float rowA = 94.5f, rowB = 113.f;
		jack(Vec(6.6f, rowA), V::WARP_INPUT, "WARP");
		jack(Vec(16.4f, rowA), V::SPAN_INPUT, "SPAN");
		jack(Vec(26.3f, rowA), V::FUSE_INPUT, "FUSE");
		jack(Vec(95.6f, rowA), V::CELL_INPUT, "CELL");
		jack(Vec(105.5f, rowA), V::MIX_INPUT, "MIX");
		jack(Vec(115.3f, rowA), V::FEED_INPUT, "FEED");

		jack(Vec(6.6f, rowB), V::IN_L_INPUT, "L");
		jack(Vec(16.4f, rowB), V::IN_R_INPUT, "R");
		jack(Vec(26.3f, rowB), V::VCA_INPUT, "VCA");
		jack(Vec(41.f, rowB), V::MORPH_INPUT, "MORPH");
		jack(Vec(51.f, rowB), V::BASIS_INPUT, "BASIS");
		jack(Vec(61.f, rowB), V::FIELD_INPUT, "FIELD");
		jack(Vec(71.f, rowB), V::TIME_INPUT, "TIME");
		jack(Vec(81.f, rowB), V::FORM_INPUT, "FORM");
		jack(Vec(95.6f, rowB), V::SCAN_INPUT, "SCAN");
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(105.5f, rowB)), module, V::OUT_L_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(115.3f, rowB)), module, V::OUT_R_OUTPUT));
		decor->labels.push_back({Vec(105.5f, rowB + 6.3f), "L", 6.5f, false});
		decor->labels.push_back({Vec(115.3f, rowB + 6.3f), "R", 6.5f, false});
		// L -> R normalling arrows
		decor->arrows.push_back({Vec(11.5f, rowB + 6.3f), Vec(1.f, 0.f)});
		decor->arrows.push_back({Vec(110.4f, rowB + 6.3f), Vec(1.f, 0.f)});

		// Level meters above the audio jacks
		meter(Vec(11.5f, 105.9f), 0);
		meter(Vec(110.4f, 105.9f), 2);

		// Plus/minus marks for the attenuverters
		for (Vec p : {Vec(30.5f, 66.5f), Vec(91.4f, 66.5f), Vec(43.6f, 88.f), Vec(61.f, 88.f), Vec(78.4f, 88.f)}) {
			decor->labels.push_back({p.plus(Vec(-5.4f, 3.6f)), "-", 7.f, false});
			decor->labels.push_back({p.plus(Vec(5.4f, 3.6f)), "+", 7.f, false});
		}
	}

	template <class TKnob>
	void knob(Vec pos, int paramId, const char* label, float labelDy, bool boxed = false) {
		addParam(createParamCentered<TKnob>(mm2px(pos), module, paramId));
		decor->labels.push_back({pos.plus(Vec(0.f, labelDy)), label, 8.f, boxed});
	}

	void atten(Vec pos, int paramId) {
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(pos), module, paramId));
	}

	void jack(Vec pos, int inputId, const char* label) {
		addInput(createInputCentered<PJ301MPort>(mm2px(pos), module, inputId));
		decor->labels.push_back({pos.plus(Vec(0.f, 6.3f)), label, 6.5f, false});
	}

	void addWindow(Vec center, int index) {
		auto* w = new ValueWindow;
		w->module = dynamic_cast<Vicks*>(module);
		w->index = index;
		w->box.size = mm2px(Vec(8.f, 2.8f));
		w->box.pos = mm2px(center).minus(w->box.size.div(2.f));
		addChild(w);
	}

	void meter(Vec center, int first) {
		auto* w = new LevelMeter;
		w->module = dynamic_cast<Vicks*>(module);
		w->first = first;
		w->box.size = mm2px(Vec(14.f, 2.6f));
		w->box.pos = mm2px(center).minus(w->box.size.div(2.f));
		addChild(w);
	}

	void modeLabel(Vec center, int index, const char* label, bool boxed) {
		auto* w = new ModeLabel;
		w->module = dynamic_cast<Vicks*>(module);
		w->index = index;
		w->label = label;
		w->boxed = boxed;
		w->box.size = mm2px(Vec(boxed ? 11.f : 10.f, 3.6f));
		w->box.pos = mm2px(center).minus(w->box.size.div(2.f));
		addChild(w);
	}

	void appendContextMenu(Menu* menu) override {
		Vicks* m = dynamic_cast<Vicks*>(module);
		if (!m)
			return;
		menu->addChild(new MenuSeparator);
		menu->addChild(createMenuLabel(string::f("Algorithm %d: %s", m->algo + 1, algoName(m->algo))));
		menu->addChild(createMenuLabel("Encoder values (current algorithm)"));
		for (int t = 0; t < Vicks::NUM_TARGETS; t++)
			menu->addChild(new TargetSlider(m, t));
		menu->addChild(createMenuItem("Randomise SEED and SCAN", "MODE + BANK", [=]() {
			m->randomizeSeedScan();
		}));
		menu->addChild(createIndexSubmenuItem("BASIS / TIME memory", {"Per algorithm", "Global"},
			[=]() {
				return m->globalBasisTime ? 1 : 0;
			},
			[=](int i) {
				float b = m->target(Vicks::T_BASIS), t = m->target(Vicks::T_TIME);
				m->globalBasisTime = (i == 1);
				m->target(Vicks::T_BASIS) = b;
				m->target(Vicks::T_TIME) = t;
			}));
		menu->addChild(createMenuItem("Reset BASIS calibration", "", [=]() {
			m->calLow = 1.f;
			m->calHigh = 3.f;
		}, m->calLow == 1.f && m->calHigh == 3.f));
	}
};


Model* modelVicks = createModel<Vicks, VicksWidget>("Vicks");
