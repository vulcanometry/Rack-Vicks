#pragma once
#include "plugin.hpp"

// DSP building blocks for Vicks: shared helpers, filters, delay lines, the
// frequency shifter and the procedurally generated wavetable bank.

namespace vicks {

// Rational tanh approximation, exact saturation at |x| >= 3.
inline float fastTanh(float x) {
	x = clamp(x, -3.f, 3.f);
	float x2 = x * x;
	return x * (27.f + x2) / (27.f + 9.f * x2);
}

// Soft clipper that is linear for small signals and saturates at +/-limit
inline float softClip(float x, float limit) {
	return limit * fastTanh(x / limit);
}

// sin(2 pi x) for any x. Parabolic approximation with one refinement step
// (about -60 dB distortion), cheap enough to run dozens per sample.
inline float sin2pi(float x) {
	x -= std::floor(x);
	float t = 2.f * x - 1.f;
	float y = 4.f * t * (1.f - std::fabs(t));
	y = 0.225f * (y * std::fabs(y) - y) + y;
	return -y;
}

inline float wrap01(float x) {
	return x - std::floor(x);
}

// Triangle wavefolder: reflects the signal back into [-1, 1]
inline float fold(float x) {
	float t = wrap01(0.25f * x + 0.25f);
	return 1.f - 4.f * std::fabs(t - 0.5f);
}

inline float polyBlep(float t, float dt) {
	if (t < dt) {
		t /= dt;
		return t + t - t * t - 1.f;
	}
	if (t > 1.f - dt) {
		t = (t - 1.f) / dt;
		return t * t + t + t + 1.f;
	}
	return 0.f;
}

inline uint32_t hash32(uint32_t x) {
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

// Deterministic random value in [0, 1) for a (key, index) pair
inline float hashUnit(uint32_t key, int32_t index) {
	return hash32(hash32(key * 0x9e3779b1u) ^ (uint32_t) index * 0x85ebca6bu) * (1.f / 4294967296.f);
}

// Smooth value noise: as `pos` moves, the output glides between random
// values placed at integer positions. Used for SEED / SCAN scanning.
inline float scanRand(uint32_t key, float pos) {
	float fl = std::floor(pos);
	int32_t i = (int32_t) fl;
	float f = pos - fl;
	f = f * f * (3.f - 2.f * f);
	float a = hashUnit(key, i);
	float b = hashUnit(key, i + 1);
	return a + (b - a) * f;
}

struct Rng {
	uint32_t s = 2463534242u;
	void seed(uint32_t v) {
		s = v ? v : 2463534242u;
	}
	// Uniform in [-1, 1)
	float uni() {
		s ^= s << 13;
		s ^= s >> 17;
		s ^= s << 5;
		return (int32_t) s * (1.f / 2147483648.f);
	}
};

// Topology-preserving state variable filter (Simper). Gives LP/BP/HP and,
// via v0 - 2k*bp, an allpass for the notch stage.
struct Svf {
	float ic1 = 0.f, ic2 = 0.f;
	float k = 1.f, a1 = 0.f, a2 = 0.f, a3 = 0.f;
	float lp = 0.f, bp = 0.f, hp = 0.f;

	// fcNorm = cutoff / sample rate
	void set(float fcNorm, float q) {
		float g = std::tan(float(M_PI) * clamp(fcNorm, 1e-5f, 0.49f));
		k = 1.f / q;
		a1 = 1.f / (1.f + g * (g + k));
		a2 = g * a1;
		a3 = g * a2;
	}
	void process(float v0) {
		float v3 = v0 - ic2;
		float v1 = a1 * ic1 + a2 * v3;
		float v2 = ic2 + a2 * ic1 + a3 * v3;
		ic1 = 2.f * v1 - ic1;
		ic2 = 2.f * v2 - ic2;
		lp = v2;
		bp = v1;
		hp = v0 - k * v1 - v2;
	}
	float allpass(float v0) {
		process(v0);
		return v0 - 2.f * k * bp;
	}
	void reset() {
		ic1 = ic2 = 0.f;
	}
};

struct DcBlock {
	float x1 = 0.f, y1 = 0.f;
	float process(float x, float r = 0.9985f) {
		float y = x - x1 + r * y1;
		x1 = x;
		y1 = y;
		return y;
	}
};

// Power-of-two circular buffer with fractional reads.
struct DelayLine {
	std::vector<float> buf;
	int mask = 0;
	int w = 0;

	void init(int minSize) {
		int size = 1;
		while (size < minSize)
			size <<= 1;
		buf.assign(size, 0.f);
		mask = size - 1;
		w = 0;
	}
	void clear() {
		std::fill(buf.begin(), buf.end(), 0.f);
	}
	int size() const {
		return mask + 1;
	}
	void write(float x) {
		buf[w] = x;
		w = (w + 1) & mask;
	}
	// Sample written `d` samples ago (d >= 1). Linear interpolation.
	float readLinear(float d) const {
		float p = (float) w - d;
		float fl = std::floor(p);
		int i = (int) fl;
		float f = p - fl;
		float a = buf[i & mask];
		float b = buf[(i + 1) & mask];
		return a + (b - a) * f;
	}
	// Cubic Hermite interpolation, for the main (audible) taps (d >= 2).
	float readHermite(float d) const {
		float p = (float) w - d;
		float fl = std::floor(p);
		int i = (int) fl;
		float f = p - fl;
		float xm1 = buf[(i - 1) & mask];
		float x0 = buf[i & mask];
		float x1 = buf[(i + 1) & mask];
		float x2 = buf[(i + 2) & mask];
		float c1 = 0.5f * (x1 - xm1);
		float c2 = xm1 - 2.5f * x0 + 2.f * x1 - 0.5f * x2;
		float c3 = 0.5f * (x2 - xm1) + 1.5f * (x0 - x1);
		return ((c3 * f + c2) * f + c1) * f + x0;
	}
};

// Schroeder allpass built on a fractional delay line
struct Diffuser {
	DelayLine dl;
	float process(float x, float d, float g) {
		float z = dl.readLinear(d);
		float v = x + g * z;
		dl.write(v);
		return z - g * v;
	}
};

// IIR Hilbert transformer (two allpass chains with a 90 degree phase
// difference across almost the whole band).
struct Hilbert {
	float xr[4][2] = {}, yr[4][2] = {};
	float xi[4][2] = {}, yi[4][2] = {};
	float imDelay = 0.f;

	static float stage(float x, float a, float* xs, float* ys) {
		// y[n] = a^2 (x[n] + y[n-2]) - x[n-2]
		float c = a * a;
		float y = c * (x + ys[1]) - xs[1];
		xs[1] = xs[0];
		xs[0] = x;
		ys[1] = ys[0];
		ys[0] = y;
		return y;
	}
	void process(float x, float* re, float* im) {
		static const float A_RE[4] = {0.4021921162426f, 0.8561710882420f, 0.9722909545651f, 0.9952884791278f};
		static const float A_IM[4] = {0.6923878f, 0.9360654322959f, 0.9882295226860f, 0.9987488452737f};
		float r = x, i = x;
		for (int s = 0; s < 4; s++) {
			r = stage(r, A_RE[s], xr[s], yr[s]);
			i = stage(i, A_IM[s], xi[s], yi[s]);
		}
		*re = r;
		*im = imDelay;
		imDelay = i;
	}
};

struct OnePole {
	float y = 0.f;
	float process(float x, float coef) {
		y += (x - y) * coef;
		return y;
	}
};


// Procedural wavetable bank: SETS tables of FRAMES frames each, every frame
// stored as band-limited mip levels (one per octave). Shared by all instances.
struct WaveBank {
	static const int N = 1024;
	static const int STRIDE = N + 1; // guard sample for interpolation
	static const int MIPS = 10;
	static const int FRAMES = 8;
	static const int SETS = 16;
	std::vector<float> data;

	static int harmonics(int mip) {
		return std::min(480, 512 >> mip);
	}

	const float* table(int set, int frame, int mip) const {
		return &data[(size_t) ((set * FRAMES + frame) * MIPS + mip) * STRIDE];
	}

	// Smallest mip level whose top harmonic stays below ~0.45 fs for phase increment dt
	static int mipFor(float dt) {
		float hmax = 0.45f / std::max(dt, 1e-7f);
		int m = 0;
		while (m < MIPS - 1 && harmonics(m) > hmax)
			m++;
		return m;
	}

	static float spectrum(int set, int h, float t, float* phase) {
		const uint32_t key = 7919u * (set + 1);
		const int family = set % 8;
		const float r0 = hashUnit(key, 1), r1 = hashUnit(key, 2), r2 = hashUnit(key, 3);
		const float fh = (float) h;
		*phase = 0.f;
		switch (family) {
			case 0: {
				// Saw with a moving formant peak
				float c = 2.f + 40.f * (r0 + (r1 - r0) * t);
				float w = 2.f + 6.f * r2;
				float d = (fh - c) / w;
				return (0.25f + 2.5f * std::exp(-d * d)) / fh;
			}
			case 1: {
				// Pulse width sweep, square to narrow pulse
				float pw = 0.5f - (0.3f + 0.17f * r0) * t;
				return std::fabs(std::sin(float(M_PI) * fh * pw)) / fh;
			}
			case 2: {
				// Sparse, bell-like partials that thicken across the table
				if (h > 96)
					return 0.f;
				float density = 0.06f + 0.5f * t;
				if (h > 1 && hashUnit(key + 11u, h) > density)
					return 0.f;
				*phase = hashUnit(key + 13u, h);
				return 1.f / std::sqrt(fh);
			}
			case 3: {
				// Sync-like resonance moving up the spectrum
				float k = 1.f + t * (6.f + 18.f * r0);
				return 1.f / (1.f + 0.6f * std::fabs(fh - k)) / std::sqrt(fh);
			}
			case 4: {
				// Saw through a comb whose spacing widens
				float sp = 0.02f + t * (0.15f + 0.2f * r0);
				return (0.5f + 0.5f * std::cos(2.f * float(M_PI) * fh * sp)) / fh;
			}
			case 5: {
				// Square to saw to even-heavy
				float even = 1.6f * t;
				return ((h % 2) ? 1.f : even) / fh;
			}
			case 6: {
				// Two formants gliding between random vowels
				float f1 = 2.f + 8.f * (r0 + (r1 - r0) * t);
				float f2 = 9.f + 25.f * (r1 + (r2 - r1) * t);
				float d1 = (fh - f1) / 1.8f;
				float d2 = (fh - f2) / 4.f;
				return (std::exp(-d1 * d1) + 0.6f * std::exp(-d2 * d2)) / std::sqrt(fh) + 0.03f / fh;
			}
			default: {
				// Random partial amplitudes, tilt opening up across the table
				float a = hashUnit(key + 17u, h);
				*phase = hashUnit(key + 19u, h);
				return a * a * std::pow(fh, -(1.6f - 1.1f * t));
			}
		}
	}

	WaveBank() {
		data.assign((size_t) SETS * FRAMES * MIPS * STRIDE, 0.f);
		std::vector<float> sine(N);
		for (int i = 0; i < N; i++)
			sine[i] = std::sin(2.0 * M_PI * i / N);
		std::vector<float> acc(N);

		for (int set = 0; set < SETS; set++) {
			for (int frame = 0; frame < FRAMES; frame++) {
				const float t = (float) frame / (FRAMES - 1);
				std::fill(acc.begin(), acc.end(), 0.f);
				// Add harmonics one at a time; snapshot each mip level as its limit is reached
				int mip = MIPS - 1;
				for (int h = 1; h <= harmonics(0); h++) {
					float ph;
					float a = spectrum(set, h, t, &ph);
					if (a > 1e-6f) {
						int off = (int) (ph * N);
						for (int i = 0; i < N; i++)
							acc[i] += a * sine[(h * i + off) & (N - 1)];
					}
					while (mip >= 0 && harmonics(mip) == h) {
						float* dst = &data[(size_t) ((set * FRAMES + frame) * MIPS + mip) * STRIDE];
						std::copy(acc.begin(), acc.end(), dst);
						dst[N] = dst[0];
						mip--;
					}
				}
				// Normalise all mips of this frame by the full-band peak
				const float* full = table(set, frame, 0);
				float peak = 1e-6f;
				for (int i = 0; i < N; i++)
					peak = std::max(peak, std::fabs(full[i]));
				for (int m = 0; m < MIPS; m++) {
					float* dst = &data[(size_t) ((set * FRAMES + frame) * MIPS + m) * STRIDE];
					for (int i = 0; i <= N; i++)
						dst[i] /= peak;
				}
			}
		}
	}

	static const WaveBank& get() {
		static WaveBank bank;
		return bank;
	}
};

inline float readTable(const float* t, float phase) {
	float p = phase * WaveBank::N;
	int i = std::min((int) p, WaveBank::N - 1);
	float f = p - i;
	return t[i] + (t[i + 1] - t[i]) * f;
}

} // namespace vicks
