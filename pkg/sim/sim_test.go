package sim

import (
	"math"
	"testing"

	"github.com/guoyyit/radar-target-detection-go/pkg/frame"
)

func header() frame.Header {
	return frame.Header{
		SampleRate:      10e6,
		ChirpSlope:      20e12,
		CarrierFreq:     77e9,
		PulseInterval:   40e-6,
		PulseCount:      16,
		SamplesPerPulse: 64,
	}
}

func TestFrameDeterministic(t *testing.T) {
	cfg := Config{
		Header:     header(),
		Targets:    []TargetSpec{{Range: 30, Velocity: 5, Amplitude: 1, Phase: 0.3}},
		NoiseSigma: 0.2,
		Seed:       42,
	}
	g1, _ := New(cfg)
	g2, _ := New(cfg)
	a, _ := g1.Frame(3)
	b, _ := g2.Frame(3)
	for p := range a.I {
		for n := range a.I[p] {
			if a.I[p][n] != b.I[p][n] || a.Q[p][n] != b.Q[p][n] {
				t.Fatalf("frame not reproducible at %d,%d", p, n)
			}
		}
	}
	c, _ := g1.Frame(4)
	if c.I[0][0] == a.I[0][0] && c.Q[0][0] == a.Q[0][0] {
		// With noise and moving target this could coincidentally equal only
		// for the noise stream; compare many samples instead.
		same := true
		for p := 0; p < 2; p++ {
			for n := 0; n < 16; n++ {
				if c.I[p][n] != a.I[p][n] {
					same = false
				}
			}
		}
		if same {
			t.Fatal("frames for different seq unexpectedly identical")
		}
	}
}

func TestInvalidConfig(t *testing.T) {
	cfg := Config{Header: header(), NoiseSigma: -1}
	if _, err := New(cfg); err == nil {
		t.Fatal("negative noise must fail")
	}
	cfg = Config{}
	if _, err := New(cfg); err == nil {
		t.Fatal("zero header must fail")
	}
}

func TestBeatFrequencyMatchesRange(t *testing.T) {
	h := header()
	targetRange := 25.0
	cfg := Config{
		Header:  h,
		Targets: []TargetSpec{{Range: targetRange, Velocity: 0, Amplitude: 1}},
	}
	g, _ := New(cfg)
	f, _ := g.Frame(0)

	// Estimate the beat frequency of pulse 0 via correlation against complex
	// exponentials; use the DFT bin grid directly (no padding).
	bestBin, bestPow := -1, math.Inf(-1)
	for k := 0; k < h.SamplesPerPulse; k++ {
		var re, im float64
		for n := 0; n < h.SamplesPerPulse; n++ {
			ph := 2 * math.Pi * float64(k) * float64(n) / float64(h.SamplesPerPulse)
			re += f.I[0][n]*math.Cos(ph) + f.Q[0][n]*math.Sin(ph)
			im += -f.I[0][n]*math.Sin(ph) + f.Q[0][n]*math.Cos(ph)
		}
		pw := re*re + im*im
		if pw > bestPow {
			bestPow, bestBin = pw, k
		}
	}
	binHz := h.SampleRate / float64(h.SamplesPerPulse)
	estRange := float64(bestBin) * binHz * c0 / (2 * h.ChirpSlope)
	if math.Abs(estRange-targetRange) > 2*binHz*c0/(2*h.ChirpSlope) {
		t.Fatalf("range estimate %v, want ~%v (bin %d)", estRange, targetRange, bestBin)
	}
}
